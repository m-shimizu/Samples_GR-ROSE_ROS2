/* GR-ROSE Sketch template V1.04 */
#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

#include "Dynamixel4TPIP3_P2.h"

DM_ServoInfo	ServoInfo[] = { 
	/*SRV 0*/{14 , +1,  00, DM_KIND_MX}, /*SRV 1*/{12 , +1,  00, DM_KIND_MX} };

#ifndef numof
#define numof(X) (sizeof(X)/sizeof(X[0]))
#endif

void loop2(void *pvParameters);

void setup() {
  // Initialize the LED pin
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // Serial output to USB
  Serial.begin(9600);

  // Initialize the Dynamixel and the RS485 port
  DM_Init_RS485(2000000);
  DM_Init_Dynamixel(numof(ServoInfo), ServoInfo);

  DM_cmd_Status_Level(0, 1);
  DM_cmd_Status_Level(1, 1);
  DM_cmd_Torque_Enable(0, 1);
  DM_cmd_Torque_Enable(1, 1);
	DM_set_limit_angle(0, -180, 180);
	DM_set_limit_angle(1, -180, 180);

  digitalWrite(PIN_LED1, 1);

	// loop2 task creation for sensing
	xTaskCreate(loop2, "LOOP2", 512, NULL, 2, NULL);
}

void loop() {
  static char msg[20];
	static float pos=-180;
	pos += 10;
	if(pos > 180)
	  pos = -180;
	DM_set_angle(0, pos);
	sprintf(msg, "Angle[0] = %d", pos);
	Serial.println(msg);
  digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
  DM_cmd_LED(0, digitalRead(PIN_LED1));
	vTaskDelay(500);
}

void loop2(void *pvParameters) {
	while (1) {
	  static char msg[20];
		static float pos=+180;
		pos -= 20;
		if(pos < -180)
		  pos = +180;
		DM_set_angle(1, pos);
		sprintf(msg, "Angle[1] = %d", pos);
		Serial.println(msg);
	  digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
	  DM_cmd_LED(1, digitalRead(PIN_LED2));
		vTaskDelay(500);
	}
}
