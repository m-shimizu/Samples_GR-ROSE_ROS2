// Copyright 2018 eSOL Co.,Ltd.
// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <Ethernet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>

#include "ros2_msg/Ros2String.h"
#include "ros2_msg/Twist.h"

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

// Micro-XRCE-DDS-Client key identifier of the client.
// If multiple clients connect to the same agent, each client must have a different key.
#define CLIENT_KEY      0xCCCCDDDD

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_TCP_TRANSPORT_MTU * STREAM_HISTORY

byte mac[] = { 0x74, 0x90, 0x50, 0x00, 0x79, 0x03 };
IPAddress ip(192, 168, 2, 52);

uxrSession session;
uxrTCPTransport transport;
uxrTCPPlatform tcp_platform;
uxrObjectId datawriter_id;  // This valiable was globalized to publish topics in a call back function.
uint16_t read_data_req;
uxrStreamId output_stream;
uxrStreamId input_stream;
bool connected = true;
static char agent_ip[32] = {0};

uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
uint8_t input_reliable_stream_buffer[BUFFER_SIZE];

void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* mb, void* args);
void on_agent_found(const uxrAgentAddress* address, void* args);
static void prvUXRManagerTask(void * pvParameters);

void setup() {
    // Initialize the LED pin
    pinMode(PIN_LED1, OUTPUT);

    // Serial output to USB
    Serial.begin(9600);

    // Setting IP
    Ethernet.begin(mac, ip);

    // Wait for network configuration
    vTaskDelay(5000);

    // Discovery Agent
    Serial.println("Discovery Agent...");
    uxrAgentAddress chosen;
    chosen.ip = agent_ip;
    
    // Try forever until Agent is found
    uxr_discovery_agents_default(INT_MAX, 1000, on_agent_found, &chosen);

    Serial.print("Chosen agent => ip: ");
    Serial.print(chosen.ip);
    Serial.print(", port: ");
    Serial.println(chosen.port);

    // Transport
    if (!uxr_init_tcp_transport(&transport, &tcp_platform, chosen.ip, chosen.port))
    {
        Serial.println("Error at create transport.");
        return;
    }

    // Session
    uxr_init_session(&session, &transport.comm, CLIENT_KEY);
    uxr_set_topic_callback(&session, on_topic, NULL);
    if (!uxr_create_session(&session))
    {
        Serial.println("Error at create session.");
        return;
    }

    // Streams
    output_stream = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);
    input_stream = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                      "<participant>"
                                          "<rtps>"
                                              "<name>default_xrce_participant</name>"
                                          "</rtps>"
                                      "</participant>"
                                  "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, output_stream, participant_id, 0, participant_xml, UXR_REPLACE);

    // Preparing a publisher and topic.
    uxrObjectId pub_topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* pub_topic_xml = "<dds>"
                                    "<topic>"
                                        "<name>rt/gr/status</name>"
                                        "<dataType>std_msgs::msg::dds_::String_</dataType>"
                                    "</topic>"
                                "</dds>";
    uint16_t pub_topic_req = uxr_buffer_create_topic_xml(&session, output_stream, pub_topic_id, participant_id, pub_topic_xml, UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, output_stream, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

                datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
                                     "<data_writer>"
                                         "<topic>"
                                             "<kind>NO_KEY</kind>"
                                             "<name>rt/gr/status</name>"
                                             "<dataType>std_msgs::msg::dds_::String_</dataType>"
                                         "</topic>"
                                     "</data_writer>"
                                 "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, output_stream, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);

    // Preparing a subscriber and topic.
    uxrObjectId sub_topic_id = uxr_object_id(0x02, UXR_TOPIC_ID);  // <= IMPORTANT: Same UXR_XXXX_ID must have different serial number. 
    const char* sub_topic_xml = "<dds>"
                                    "<topic>"
                                        "<name>rt/gr/cmd_vel</name>"
                                        "<dataType>geometry_msgs::msg::dds_::Twist_</dataType>"
                                    "</topic>"
                                "</dds>";
    uint16_t sub_topic_req = uxr_buffer_create_topic_xml(&session, output_stream, sub_topic_id, participant_id, sub_topic_xml, UXR_REPLACE);

    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "";
    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session, output_stream, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml = "<dds>"
                                     "<data_reader>"
                                         "<topic>"
                                             "<kind>NO_KEY</kind>"
                                             "<name>rt/gr/cmd_vel</name>"
                                             "<dataType>geometry_msgs::msg::dds_::Twist_</dataType>"
                                         "</topic>"
                                     "</data_reader>"
                                 "</dds>";
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(&session, output_stream, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

#define _numof(X) (sizeof(X)/sizeof(X[0]))

    // Send create entities message and wait its status
    uint16_t requests[] = {participant_req, pub_topic_req, sub_topic_req, publisher_req, subscriber_req, datawriter_req, datareader_req};
    uint8_t status[_numof(requests)];
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, _numof(requests)))
    {
        Serial.print("Error at create entities: participant: ");
        Serial.print(status[0]);
        Serial.print(" publisher topic: ");
        Serial.print(status[1]);
        Serial.print(" subscriber topic: ");
        Serial.print(status[2]);
        Serial.print(" publisher: ");
        Serial.print(status[3]);
        Serial.print(" subscriber: ");
        Serial.print(status[4]);
        Serial.print(" datawriter: ");
        Serial.println(status[5]);
        Serial.print(" datareader: ");
        Serial.println(status[6]);
    }

    // Request topics
    uxrDeliveryControl delivery_control = {0};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    read_data_req = uxr_buffer_request_data(&session, output_stream, datareader_id, input_stream, &delivery_control);

    xTaskCreate(prvUXRManagerTask, "Pub and Sub", configMINIMAL_STACK_SIZE * 5, NULL, 2, NULL);
}

void loop() {
    // Do nothing
    vTaskDelay(10000);
}

void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* mb, void* args) {
    (void) session; (void) object_id; (void) request_id; (void) stream_id;

    // Processing twist message.
	Twist twist;
	Twist_deserialize_topic(mb, &twist);
	Serial.print("Received topic: ");
    Serial.print("{angular:{x:");
    Serial.print(twist.angular.x);
    Serial.print(", y:");
    Serial.print(twist.angular.y);
    Serial.print(", z:");
    Serial.print(twist.angular.z);
    Serial.print("}, linear:{x:");
    Serial.print(twist.linear.x);
    Serial.print(", y:");
    Serial.print(twist.linear.y);
    Serial.print(", z:");
    Serial.print(twist.linear.z);
    Serial.println("}}");

    // Reporting status.
	Ros2String status;
    sprintf(status.data, "GR Status: XXXXXXX"); // Set the status message.
    ucdrBuffer buffer;
    uint32_t topic_size = Ros2String_size_of_topic(&status, 0);
    uxr_prepare_output_stream(session, output_stream, datawriter_id, &buffer, topic_size);
    Ros2String_serialize_topic(&buffer, &status); // Publishing a topic.
    Serial.println(status.data); // Print the message to the usb serial port for debugging.

/*
	Ros2String topic;
    Ros2String_deserialize_topic(mb, &topic);

    Serial.print("Received topic: ");
    Serial.println(topic.data);
*/
    // Toggle the heartbeat LED
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
}

void on_agent_found(const uxrAgentAddress* address, void* args) {
    uxrAgentAddress* agent = (uxrAgentAddress*)args;

    Serial.print("Found agent => ip: ");
    Serial.print(address->ip);
    Serial.print(", port: ");
    Serial.println(address->port);

    memcpy((void*)(agent->ip), address->ip, 32);
    agent->port = address->port;
}

static void prvUXRManagerTask(void * pvParameters) {
    // Loop as long as client is connected to agent.
    while (connected)
    {
        // Receive topics
        if (connected)
        {
            uint8_t read_data_status;
            connected = uxr_run_session_until_all_status(&session, UXR_TIMEOUT_INF, &read_data_req, &read_data_status, 1);
        }
        else
        {
            Serial.println("connection error");
        }

        vTaskDelay(1);
    }

    // Delete resources
    (void) uxr_delete_session(&session);
    (void) uxr_close_tcp_transport(&transport);
    
    // Delete this task
    vTaskDelete(NULL);
}