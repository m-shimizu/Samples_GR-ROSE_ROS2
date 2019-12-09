/*---------------------------------------------------------------*
 | Dynamixel Protocol 2.0                     2017-06-10         |
 |                                             By SHIMIZU Lab.   |
 |  COMPILER: VisualStudio2010                    Chukyo Univ.   |
 |  TARGET  : TPIP3                                              |
 *---------------------------------------------------------------*/
#include <stdio.h>
//#include "TPIP3.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>


/************************************/
/*	Defines for Debugging           */
/************************************/

#define DM_DEBUG

#ifdef DM_DEBUG
#define	DEBUG_MSG1(X)	Serial.println(X)
#define DEBUG_MSG2(X,Y)	deb_mes(X,Y)
#define DEBUG_MSGE(X,Y)	;
#define DEBUG_MSG3(X,Y,Z)	deb_mes(X,Y,Z)
/*
#define	DEBUG_MSG1(X)	printf(X)
#define DEBUG_MSG2(X,Y)	printf(X,Y)
#define DEBUG_MSG3(X,Y,Z)	printf(X,Y,Z)
*/
#else
#define DEBUG_MSG1(X) ;
#define DEBUG_MSG2(X,Y) ;
#define DEBUG_MSGE(X,Y)	;
#define DEBUG_MSG3(X,Y,Z) ;
#endif

// ########################################################################  message window for debugging
//void deb_mes(char* text);
//void deb_mes(char* title, int dat);
//void deb_mes(char* title, float dat);
//void deb_mes(char* title1, float dat1, char* title2, float dat2);

static inline void deb_mes(char * text)
{
    Serial.println(text);
/*
	char out_test[100];
	if (text==NULL)
	{
		MessageBox(NULL , TEXT("NULLPOINT"),TEXT("メッセージボックス") , MB_OK);
	}
	else
	{	
		wsprintf(out_test,"%s",text);
		MessageBox(NULL , out_test ,TEXT("メッセージボックス") , MB_OK);
	}
	*/
}

static inline void deb_mes(char* title, int dat)
{
	char msg[100];
	sprintf(msg, "%s%d", title, dat);
	deb_mes(msg);
}

static inline void deb_mes(char* title, char dat)
{
	char msg[100];
	sprintf(msg, "%s%d", title, dat);
	deb_mes(msg);
}

static inline void deb_mes(char* title, const char* dat)
{
	char msg[100];
	sprintf(msg, "%s%s", title, dat);
	deb_mes(msg);
}

static inline void deb_mes(char* title, float dat)
{
	char msg[100];
	sprintf(msg, "%s%f", title, dat);
	deb_mes(msg);
}

static inline void deb_mes(char* title1, float dat1, char* title2, float dat2)
{
	char msg[100];
	sprintf(msg, "%s%f, %s%f", title1, dat1, title2, dat2);
	deb_mes(msg);
}

/*******************************************************************************
*
*  TPIP3 UDP Port
*
*******************************************************************************/
//#include "TPIP3.h"
//#include "TPUD_13dll.h"
//w32udp* G_RS485_UDP;

/* TPIP3 config file
$SEN_SET, 9FFF // Sensor enable bit 
$COM_OFF, 0000 // 通信OFF時の設定　
$RS232_1, 9000, 115200, 8N1 // RS232 #1 UDPport, speed, bit Parity stopbit
$PWM_MODE, 0000
$CONNECT_TYPE, 0                            <= IMPORTANT!! RS232C CONNECTION ONLY!!
$CBOARD_NUM, 1
$VIDEO_FMT, 0
$VIDEO, 0, 0, 4000
$RS485_SET, 7001, 3000000, 8, 0, 1, 50       <= IMPORTANT!! 3Mbps!!
$RS232_2, 9001, 57600, 8N1 // RS232 #2 UDPport, speed, bit Parity stopbit*/

void	DM_Init_RS485(unsigned int baudrate)
{
    Serial7.begin(baudrate, SERIAL_8N1);
    Serial7.direction(OUTPUT);
    Serial7.setTimeout(50);
    vTaskDelay(1000);
/*
	G_RS485_UDP = new w32udp("UDP_C");
	G_RS485_UDP->open(host_ip, port_number, 50); // wait forever
*/
}

void	DM_Destroy_RS485(void)
{
//	G_RS485_UDP->close();
//	delete G_RS485_UDP;
}

inline void	send_rs485(unsigned char* dat, int num)
{
    Serial7.flush();
    vTaskDelay(20);
    Serial7.write((const uint8_t*)dat, (uint16_t)num);
//	G_RS485_UDP->send((void*)dat, num);
}

inline int	receive_rs485(unsigned char* dat, int dat_max_size)
{
//	return G_RS485_UDP->recv_rt((void*)dat, dat_max_size, 50);  // wait forever
}

/*******************************************************************************
*
*  Dynamixel DX116 and Lator
*
*******************************************************************************/
#include "Dynamixel4TPIP3_P2.h"

static int	 MAX_MOTOR_NUM;
static DM_ServoInfo* DM_SI;

//                                     {    MX}
int	DM_Max_Angle_in_Binary_from_kind[]={  4096}; // Max Angle in Binary
int DM_Max_Angle_in_Degree_from_kind[]={   360}; // Max Angle in Degree

void	DM_Init_Dynamixel(int _motors, DM_ServoInfo* dsip)
{
	MAX_MOTOR_NUM = _motors;
	DM_SI = dsip;
}

#ifndef _MIN
#define _MIN(X,Y) (((X)<(Y))?(X):(Y))
#endif
#ifndef _MAX
#define _MAX(X,Y) (((X)>(Y))?(X):(Y))
#endif

struct DM_CMD
{
	BYTE	HEAD[4];
	BYTE	ID;
	BYTE  LEN[2]; // No parameters, LENGTH = 3.
	BYTE	INSTRUCTION;
	BYTE	PARAMETER[30];
	      DM_CMD(void){ HEAD[0] = HEAD[1] = 0xFF; HEAD[2] = 0xFD; HEAD[3] = 0; }
	void  _WORD(int indx, WORD*dat) { PARAMETER[indx + 0] = *dat & 0xFF; PARAMETER[indx + 1] = (*dat >> 8) & 0xFF; }
	WORD  _WORD(int indx){ return (WORD)PARAMETER[indx] + (WORD)PARAMETER[indx + 1] * 0x100; }
	void  _LWORD(int indx, LWORD*dat) { PARAMETER[indx + 0] = *dat & 0xFF; PARAMETER[indx + 1] = (*dat >> 8) & 0xFF; PARAMETER[indx + 2] = (*dat >> 16) & 0xFF; PARAMETER[indx + 3] = (*dat >> 24) & 0xFF; }
	LWORD _LWORD(int indx){ return (LWORD)PARAMETER[indx] + (LWORD)PARAMETER[indx + 1] * 0x100 + (LWORD)PARAMETER[indx + 2] * 0x10000 + (LWORD)PARAMETER[indx + 3] * 0x1000000; }
	void  LENGTH(WORD _len){ _len += 3;  LEN[0] = _len & 0xFF; LEN[1] = (_len >> 8) & 0xFF; }
	WORD  LENGTH(void){ return (WORD)LEN[0] + (WORD)LEN[1] * 0x100; }
	void  ADDRESS(WORD _adr){ _WORD(0, &_adr); }
	WORD  ADDRESS(void){ return _WORD(0); }
	void  send(void)
	{
		WORD crc;
		crc = update_crc(0, (unsigned char*)this, (this->LENGTH()) + 5);
		this->PARAMETER[(this->LENGTH()) - 3] = crc & 0x00ff;
		this->PARAMETER[(this->LENGTH()) - 2] = (crc >> 8) & 0x00ff;
		send_rs485((unsigned char*)this, (this->LENGTH()) + 7);
		//	for(dat = 0; dat < 200; dat++); // Wait for slow device.
	}
	unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
	{// update_crc came from http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/crc.htm
		unsigned short i, j;
		unsigned short crc_table[256] = {
			0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
			0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
			0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
			0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
			0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
			0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
			0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
			0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
			0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
			0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
			0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
			0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
			0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
			0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
			0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
			0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
			0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
			0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
			0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
			0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
			0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
			0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
			0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
			0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
			0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
			0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
			0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
			0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
			0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
			0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
			0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
			0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
		};
		for (j = 0; j < data_blk_size; j++)
		{
			i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
			crc_accum = (crc_accum << 8) ^ crc_table[i];
		}
		return crc_accum;
	}
};

typedef struct DM_CMD	DM_CMD;

inline void	DM_write(DM_CMD* cmd)
{
	// THIS IS A STUB.
	// USE cmd.send() instead of calling this function.
}

/*
inline int	DM_read_1_BYTE(BYTE *b) // This never return untill get 1 byte from rs485.
{
	int num;
	*b=123;
//	do
//	{
		num = receive_rs485(b, 1);
//		for(num = 0; num < 200; num++); // Wait for slow device.

//		deb_mes("READ,num:",num,"READ,dat:",*b);
//	}
//	while(0 != num);
		return 0;
//	else
//		return -1;
//	{
//		deb_mes("READ,num:",num);
//		deb_mes("READ,dat:",*b);
//	}
//	return 0;
}

inline int	DM_read_wait_for_FFFF(void)
{
	BYTE	dat;
	int i;
	for(i = 0; i < 2; i++)
		if(-1 == DM_read_1_BYTE(&dat))
			return -1;
		else if(0xFF != dat)
			i = 0;
	return 0;
}

inline int	DM_read_parameters(DM_CMD* cmd)
{
	BYTE	dat;
	int	i;
	for(i = 0; i < cmd->LENGTH - 2; i++)
		if(-1 == DM_read_1_BYTE(&dat))
			return -1;
		else
		{
			cmd->PARAMETER[i] = dat;
//			deb_mes("dat:", dat);
		}
	return 0;
}
*/

inline int	DM_read_nBYTE(BYTE *buf, int num) // This never return untill get 1 byte from rs485.
{
	return receive_rs485(buf, num);
}

inline BYTE* DM_check_header(BYTE* dat)
{
	if (0xFF == *dat++)
		if (0xFF == *dat++)
			if (0xFD == *dat++)
				return dat;
	return (BYTE*)NULL;
}

inline int	DM_read_core(DM_CMD* cmd)
{
static	BYTE	dat[128];
	int		datlen;
	BYTE*	readpoint;
	datlen = DM_read_nBYTE(dat, sizeof(dat));
//deb_mes("datlen:", datlen);
	if(11 > datlen) return -1;
//deb_mes("datlen is ok");
	if((BYTE*)NULL == (readpoint = DM_check_header(&dat[0]))) return -1;
//deb_mes("Header is ok");
	if(cmd->ID != *readpoint++) return -1;
//deb_mes("ID is ok");
	cmd->LENGTH(*readpoint++);
//deb_mes("LENGTH:", cmd->LENGTH());
	readpoint++;
	cmd->INSTRUCTION = *readpoint++;
//deb_mes("ERROR:", cmd->INSTRUCTION);
	for(int i = 0; i < cmd->LENGTH() - 3; i++)
	{
		cmd->PARAMETER[i] = readpoint[i];
//deb_mes("PARAM:", cmd->PARAMETER[i]);
	}
/*	if(-1 == DM_read_wait_for_FFFF(dat))	return -1;
	if(-1 == DM_read_1_BYTE(&cmd->ID))	return -1;
	if(-1 == DM_read_1_BYTE(&cmd->LENGTH))	return -1;
	if(-1 == DM_read_1_BYTE(&cmd->INSTRUCTION))	return -1;
	if(-1 == DM_read_parameters(cmd))	return -1;
	if(-1 == DM_read_1_BYTE(&dat))	return -1; */ // read checksum
//deb_mes("ID:", cmd->ID);
//deb_mes("LEN:", cmd->LENGTH);
//deb_mes("ERROR:", cmd->INSTRUCTION);
//	for(dat = 0; dat < 200; dat++); // Wait for slow device.
//	for(dat = 0; dat < 200; dat++); // Wait for slow device.
	return 0;
}

inline int	DM_read(DM_CMD* cmd)
{
	if(-1 == DM_read_core(cmd))
		return DM_read_core(cmd);
	return 0;
}

//Baud Rate 通信スピード 0x04
//Return Delay Time 遅延時間 0x05

//稼動範囲
void	DM_cmd_Operating_Angle_Limit(int num, LWORD CW_limit, LWORD CCW_limit) // CCW > CW
{
	DM_CMD	_cmd;
	LWORD   CCW_tmp=CCW_limit,CW_tmp=CW_limit;
	DEBUG_MSG1("Operating Angle Limit\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	CCW_limit = _MIN(4095, _MAX(CCW_tmp, CW_tmp));
	CW_limit  = _MAX(0,    _MIN(CCW_tmp, CW_tmp));
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(10);   //The number of parameters(BYTES)
	_cmd.ADDRESS(48);   //アドレス PARAMETER[0] and [1]
	_cmd._LWORD(2, &CCW_limit); //CCW Angle Limit PARAMETER[2],[3],[4],[5]
	_cmd._LWORD(6, &CW_limit);  // CW Angle Limit PARAMETER[6],[7],[8],[9]
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//動作制限温度
void	DM_cmd_the_Highest_Limit_Temperature(int num, BYTE max_tmp)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("the Highest Limit Temperature\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
  _cmd.ADDRESS(31);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = max_tmp & 0xff;  //制限温度の上限(100)
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//動作範囲電圧 low[V] , high[V]
void	DM_cmd_the_Lowest_Highest_Limit_Voltage(int num, float low_v, float high_v)
{
	DM_CMD	_cmd;
	WORD  low, high;
	low = _MIN(low_v, high_v) * 10;
	high = _MAX(low_v, high_v) * 10;
	DEBUG_MSG1("the Lowest(Highest) Limit Voltage\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(6);   //The number of parameters(BYTES)
	_cmd.ADDRESS(32);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &high);  //動作範囲電圧 上限(160) PARAMETER[2] and [3]
	_cmd._WORD(4, &low);  //動作範囲電圧 下限(95) PARAMETER[4] and [5]
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//最大トルク設定
void	DM_cmd_Max_Torque(int num, int trq)
{
	// No way to set a max torque in protocol 2.0
}


void	DM_cmd_Operating_Mode(int num, DM_OPERATING_MODE mode)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Operating Mode\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(11);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = mode; // PARAMETER[2]
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


void	DM_cmd_Drive_Mode(int num, DM_DRIVE_MODE mode)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Drive (Direction) Mode\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(10);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = mode; // PARAMETER[2]
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//最大PWM設定 // From 0 to 1.0
void	DM_cmd_Max_PWM(int num, float pwm)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Max PWM\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	if (pwm < 0)
		pwm = -pwm;
	pwm = _MIN(1.0, pwm);
	short int _pwm = pwm * 885;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(36);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, (WORD*)&_pwm); // PARAMETER[2] and [3]
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//最大Acceleration設定 // From 0 to 32767 * 214.577[rev/min2]
void	DM_cmd_Max_Acceleration(int num, float accel)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Max Acceleration\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	if (accel < 0)
		accel = -accel;
	LWORD _accel = _MIN(32767 * 214.577, accel) / 214.577;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(6);   //The number of parameters(BYTES)
	_cmd.ADDRESS(40);   //アドレス PARAMETER[0] and [1]
	_cmd._LWORD(2, (LWORD*)&_accel); // PARAMETER[2] and [3][4][5]
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//最大Velocity設定 // From 0rpm to 1023*0.229rpm
void	DM_cmd_Max_Velocity(int num, float vel)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Max Acceleration\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	if (vel < 0)
		vel = -vel;
	LWORD _vel = _MIN(1023 * 0.229, vel) / 0.229;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(6);   //The number of parameters(BYTES)
	_cmd.ADDRESS(44);   //アドレス PARAMETER[0] and [1]
	_cmd._LWORD(2, (LWORD*)&_vel); // PARAMETER[2] and [3][4][5]
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//Status Packetのreturn level設定 //Return Level=0:Never, 1:Only Reading, 2:Always (Level=1 is recommended)
void	DM_cmd_Status_Level(int num, BYTE level)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Status Level\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(68);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = level;  //PARAMETER[2]
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//アラームLED
void	DM_cmd_Alarm_LED(int num, int dat)
{
  // No way to set alarm LED in protocol 2.0
}

//非常停止
void	DM_cmd_Shutdown(int num, DM_SHUTDOWN dat)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Alarm Shutdown\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(63);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = dat;  //停止Error Flags. SEE DM_SHUTDOWN
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//LEDの設定
void	DM_cmd_LED(int num, BYTE dat)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("LED\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(65);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = dat;  //LED点灯Error Flags
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//RESET
void	DM_cmd_Reset(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("RESET\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 8;  //reboot
	_cmd.LENGTH(0);   //The number of parameters(BYTES)
	_cmd.send();
	//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//移動目標位置
void	DM_cmd_Goal_Position(int num, LWORD ang)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Set Goal Position\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(6);   //The number of parameters(BYTES)
	_cmd.ADDRESS(116);   //アドレス PARAMETER[0] and [1]
	_cmd._LWORD(2, &ang); // PARAMETER[2] and [3][4][5]
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//移動速度
void	DM_cmd_Goal_Velocity(int num, float rpm)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Moving Speed\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	long int _rpm = rpm / 0.229;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3;  //write
	_cmd.LENGTH(6);   //The number of parameters(BYTES)
	_cmd.ADDRESS(104);   //アドレス PARAMETER[0] and [1]
	_cmd._LWORD(2, (LWORD*)&_rpm); // PARAMETER[2][3][4][5]
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}

#define DM_READ_ERROR -100000000000L

//現在位置(read)
LWORD	DM_cmd_Present_Position(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Present Position\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return 0;
	}*/
	WORD read_size = 4;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(132);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if(-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
//deb_mes("Pos:", ((_cmd.PARAMETER[1] << 8) | _cmd.PARAMETER[0]) & DM_Max_Angle_in_Binary_srv(num));
//deb_mes("Pos:", pos);
//	DEBUG_MSG3("PositionX10[Deg](%d) = %d\r", num, (int)(pos*10));
	return _cmd._LWORD(0);
}


//現在速度(read)
LWORD	DM_cmd_Present_Velocity(int num)
{
	DM_CMD	_cmd;
	float	speed;
	DEBUG_MSG1("Present Speed\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 4;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(128);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd._LWORD(0);
}


//印加電圧(read)
WORD	DM_cmd_Present_Voltage(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Present Voltage\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 2;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(144);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd._WORD(0);
}


//内部摂氏温度(read)
BYTE	DM_cmd_Present_Temperature(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Present Temperature\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 1;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(146);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd.PARAMETER[0];
}


//現在駆動している負荷の大きさ(read)
WORD	DM_cmd_Present_Load(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Get Present Load\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 2;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(126);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd._WORD(0);
}


//現在駆動しているPWMの大きさ(read)
WORD	DM_cmd_Present_PWM(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Get Present Load\n");
	/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
	DEBUG_MSG1("Number Error\n");
	return;
	}*/
	WORD read_size = 2;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(124);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd._WORD(0);
}


//ＲＥＧとＡＣＴＩＯＮの関係
BYTE	DM_cmd_Registered_Instruction(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Registered_Instruction\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 1;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(69);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd.PARAMETER[0];
}


//自力で動いているか調べる(read)
BYTE	DM_cmd_Moving(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Moving\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	WORD read_size = 1;
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 2;  //read
	_cmd.LENGTH(4);   //The number of parameters(BYTES)
	_cmd.ADDRESS(122);   //アドレス PARAMETER[0] and [1]
	_cmd._WORD(2, &read_size);  //dataの長さ, PARAMETER[2][3]
	_cmd.send();
	if (-1 == DM_read(&_cmd))
		return DM_READ_ERROR;
	return _cmd.PARAMETER[0];
}


//ロック
void	DM_cmd_Lock(int num)
{
  // No way to set EEPROM LOCk in Protocol 2.0.
}


//Compliance Margin
void	DM_cmd_Margin(int num, int CW_Margin, int CCW_Margin)
{
}


//Compliance Slope
void	DM_cmd_Slope(int num, int CW_Slope, int CCW_Slope)
{
}


//モータへの最小限の電流量
void	DM_cmd_Punch(int num, int punch)
{
}


//Status Packetをreturnさせる
void	DM_cmd_ping(int num)
{
	DM_CMD	_cmd;
	DEBUG_MSG1("ping\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
  _cmd.ID = DM_SI[num].id; //ID番号
	_cmd.INSTRUCTION = 1; // ping=return status packet
	_cmd.LENGTH(0);   //The number of parameters(BYTES)
	_cmd.send();
	DM_read(&_cmd);
//	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//Motor Torque ON/OFF
void	DM_cmd_Torque_Enable(int num, int on_off)
{
	DM_CMD	_cmd;
	DEBUG_MSG1((on_off)?"Torque On":"Torque off");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = DM_SI[num].id;  //ID番号
	_cmd.INSTRUCTION = 3; // write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(64);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = (on_off) ? 1 : 0;
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


// Set Motor ID
void	DM_cmd_Set_Number(int num) // !!!Every motor will be changed!!!
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Set number\n");
/*	if(!(num >= 0 && num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}*/
	_cmd.ID = 0xfe; // broadcast ID
	_cmd.INSTRUCTION = 3; // write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(7);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = num;
	_cmd.send();
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


// Set Baudrate
void	DM_cmd_Set_Baudrate_3000000(void) // !!!Every motor will be changed!!!
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Set baudrate to 3000000 bps\n");
	_cmd.ID = 0xfe; // broadcast ID
	_cmd.INSTRUCTION = 3; // write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(8);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = 5; // 3,000,000bps
	DM_write(&_cmd);
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


void	DM_cmd_Set_Baudrate_57600(void) // !!!Every motor will be changed!!!
{
	DM_CMD	_cmd;
	DEBUG_MSG1("Set baudrate to 57600 bps\n");
	_cmd.ID = 0xfe; // broadcast ID
	_cmd.INSTRUCTION = 3; // write
	_cmd.LENGTH(3);   //The number of parameters(BYTES)
	_cmd.ADDRESS(8);   //アドレス PARAMETER[0] and [1]
	_cmd.PARAMETER[2] = 1; // 57,600bps
	DM_write(&_cmd);
//	DM_read(&_cmd);
	DEBUG_MSGE("%02X\n", _cmd.ERROR);
}


//##############################################################################

#define DM_BPA_srv(X) DM_Max_Angle_in_Binary_srv(X)/DM_Max_Angle_in_Degree_srv(X)
#define DM_APB_srv(X) DM_Max_Angle_in_Degree_srv(X)/DM_Max_Angle_in_Binary_srv(X)

#define	Angle2Pos(X)	(int)((X)*DM_BPA_srv(srv_num))
#define	Angle2Pos_With_offset_center(X)	Angle2Pos(DM_SI[srv_num].sign * (X) \
											- DM_SI[srv_num].offset + DM_Max_Angle_in_Degree_srv(srv_num)/2)
#define	CheckMaxMinInBinary(X)	(((X)<0)?0:(((X)>(DM_Max_Angle_in_Binary_srv(srv_num)))?(DM_Max_Angle_in_Binary_srv(srv_num)):(X)))

void	DM_set_angle(int srv_num, float _angle) //角度関数(Center = 0 degree)
{
	int	ang;
	if(!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}
	ang = Angle2Pos_With_offset_center(_angle);
	ang = CheckMaxMinInBinary(ang);
/*	ang = Angle2Pos(DM_SI[srv_num].sign * _angle - DM_SI[srv_num].offset + DM_Max_Angle_in_Degree_srv(srv_num)/2);
	if(ang < 0)
		ang = 0;
	else if(ang > DM_Max_Angle_in_Binary_srv(srv_num))
		ang = DM_Max_Angle_in_Binary_srv(srv_num);*/
	DM_cmd_Goal_Position(srv_num, ang);
}

float DM_get_angle(int srv_num)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	return (DM_cmd_Present_Position(srv_num));
//	return (DM_cmd_Present_Position(srv_num) * DM_Max_Angle_in_Degree_srv(srv_num) / DM_Max_Angle_in_Binary_srv(srv_num)
//		+ DM_SI[srv_num].offset - DM_Max_Angle_in_Degree_srv(srv_num) / 2)* DM_SI[srv_num].sign;
}


float	DM_get_load(int srv_num)
{
	WORD load;
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	load = DM_cmd_Present_Load(srv_num);
	return (*(short int*)&load) * 0.1 * DM_SI[srv_num].sign * -1;
}


float	DM_get_pwm(int srv_num)
{
	WORD pwm;
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	pwm = DM_cmd_Present_PWM(srv_num);
	return (*(short int*)&pwm) / (float)885 * DM_SI[srv_num].sign * -1;
}


void DM_set_velocity(int srv_num, float rpm) // (rpm)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
	}
	DM_cmd_Goal_Velocity(srv_num, rpm);
}


float DM_get_velocity(int srv_num) // (rpm)
{
	LWORD	vel;
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	vel = DM_cmd_Present_Velocity(srv_num);
	return (*(long int*)&vel) * 0.229 * DM_SI[srv_num].sign * -1;
}


float DM_get_temperature(int srv_num) //内部摂氏温度(degree)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	return DM_cmd_Present_Temperature(srv_num);
}


float DM_get_voltage(int srv_num)//V
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return DM_READ_ERROR;
	}
	return DM_cmd_Present_Voltage(srv_num) * 0.1;
}


void	DM_Wheel_Mode(int srv_num)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
	}
	DM_cmd_Torque_Enable(srv_num, 0);
	DM_cmd_Operating_Mode(srv_num, DMD_VELOCITY);
	DM_cmd_Torque_Enable(srv_num, 1);
}


void	DM_Multiturn_Mode(int srv_num)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
	}
	DM_cmd_Torque_Enable(srv_num, 0);
	DM_cmd_Operating_Mode(srv_num, DMD_EXTENDED_POSITION);
	DM_cmd_Torque_Enable(srv_num, 1);
}


void	DM_Servo_Mode(int srv_num)
{
	if (!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
	}
	DM_cmd_Torque_Enable(srv_num, 0);
	DM_cmd_Operating_Mode(srv_num, DMD_POSITION);
	DM_cmd_Torque_Enable(srv_num, 1);
}


void	DM_set_limit_angle(int srv_num, float _CW_limit, float _CCW_limit)
{												 //角度関数(Center = 0 degree)
	LWORD	CW_limit, CCW_limit, tmp;
	if(!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}
	CW_limit  = Angle2Pos_With_offset_center(_CW_limit);
//	CW_limit  = CheckMaxMinInBinary(CW_limit);
	CCW_limit = Angle2Pos_With_offset_center(_CCW_limit);
//	CCW_limit = CheckMaxMinInBinary(CCW_limit);
	if(CW_limit > CCW_limit)
	{
		tmp = CCW_limit;
		CCW_limit = CW_limit;
		CW_limit  = tmp;
	}
	DM_cmd_Operating_Angle_Limit(srv_num, CW_limit, CCW_limit);
}


void	DM_Wait_Done(int srv_num)
{
	if(!(srv_num >= 0 && srv_num < MAX_MOTOR_NUM))
	{
		DEBUG_MSG1("Number Error\n");
		return;
	}
	while(DM_cmd_Moving(srv_num) != 0);
	DEBUG_MSG1("D");
}


void	DM_Power_Off_All_Servo(void)
{
	int	m;
	for(m = 0; m < MAX_MOTOR_NUM; m++)
		DM_cmd_Torque_Enable(m, 0);
}


void	DM_Power_On_All_Servo(void)
{
	int	m;
	for(m = 0; m < MAX_MOTOR_NUM; m++)
		DM_cmd_Torque_Enable(m, 1);
}


//##############################################################################

void	Set_charactor(void)
{
	int	i;
	for(i = 1; i <= 14; i++)
	{
		DM_cmd_Punch(i, 1);
		DM_cmd_Slope(i, 10, 10);
		DM_cmd_Goal_Velocity(i, 1023*0.229);
		DM_cmd_Max_Torque(i, 0x3ff);
	}
}

void	Set_speed(float spd)
{
	int	i;
	for(i = 1; i <= 14; i++)
		DM_cmd_Goal_Velocity(i, spd);
}

void	Move_to_origin(void)
{
	int	i;
	for(i = 1; i <= 14; i++)
		DM_set_angle(i, 0);
}

void	Set_motors_torque(int f)
{
	int	i;
	for(i = 1; i <= 14; i++)
		DM_cmd_Torque_Enable(i, f);
}

void	Set_motors_angle(float a)
{
	int	i;
	for(i = 1; i <= 14; i++)
		DM_set_angle(i, a);
}

void	Move_Angle(int num, int d)
{
	//DEBUG_MSG2("%d <- ", d);
	DM_set_angle(num, DM_get_angle(num) + d * 10);
}

/*
float	Receive_Angle(void)
{
	int	f, t;
	f = (SCI1_IN_DATA() == '-')?-1:+1;
	t = SCI1_IN_DATA() - '0';
	t = t * 10 + SCI1_IN_DATA() - '0';
	t = t * 10 + SCI1_IN_DATA() - '0';
	t = t * 10 + SCI1_IN_DATA() - '0';
	return (float)(f * t / 10);
}

#define _ABS(X)	(((X) < 0)?(-X):(X))

void	Send_Angle(float ang)
{
	SCI1_PRINTF("%c%04d", (ang < 0)?'-':'+', (int)(_ABS(ang)));
}
*/
