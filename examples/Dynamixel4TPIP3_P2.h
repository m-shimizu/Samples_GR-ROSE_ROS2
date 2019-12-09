/*---------------------------------------------------------------*
 | Motion Control Program                     2014-11-01         |
 |                                             By SHIMIZU Lab.   |
 |  COMPILER: VisualStudio2010                    Chukyo Univ.   |
 |  TARGET  : TPIP3                                              |
 *---------------------------------------------------------------*/
#ifndef DX116P2_H
#define DX116P2_H

//#pragma comment(lib, "TPUD_13.lib")

typedef struct DM_Servo_Information
{
	int	id;
	char	sign;
	int		offset;
	int		dm_kind;
} DM_ServoInfo;

enum DM_KIND
{
	DM_KIND_MX = 0
};

enum DM_OPERATING_MODE
{
	DMD_VELOCITY = 1,          // Wheel mode
	DMD_POSITION = 3,          // Servo motor mode
	DMD_EXTENDED_POSITION = 4, // Multi turn mode
	DMD_PWM_CONTROL = 16       // ??
};

enum DM_DRIVE_MODE
{
	DMD_NORMAL_DIRECTION = 0,  // CCW:Positive, CW:Negative
	DMD_REVERSE_DIRECTION = 1  // CCW:Negative, CW:Positive
};

enum DM_SHUTDOWN
{
	DMS_OVERLOAD = 0x20,
	DMS_E_SHOCK = 0x10,
	DMS_ENCODER = 0x08,
	DMS_OVERHEAT = 0x04,
	DMS_VOLTAGE = 0x01
};

#define	DM_Max_Angle_in_Binary_srv(X)	DM_Max_Angle_in_Binary_from_kind[DM_SI[X].dm_kind]
#define	DM_Max_Angle_in_Degree_srv(X)	DM_Max_Angle_in_Degree_from_kind[DM_SI[X].dm_kind]
#define DM_Init_ServoInfo(X) DM_Init_Dynamixel(sizeof(X)/sizeof(DM_ServoInfo), X)

#ifndef BYTE
typedef unsigned char BYTE;
#endif
#ifndef WORD
typedef unsigned short int WORD;
#endif
#ifndef LWORD
typedef unsigned long int LWORD;
#endif

extern void	DM_Init_RS485(unsigned int baudrate); // 2000000 is max
extern void DM_Destroy_RS485(void);
extern void	DM_Init_Dynamixel(int max_motor_num , DM_ServoInfo* DSI);
extern void	DM_cmd_Operating_Angle_Limit(int num, LWORD CW_limit, LWORD CCW_limit); // CCW > CW
extern void	DM_cmd_the_Highest_Limit_Temperature(int num, BYTE max_tmp);//動作制限温度 (degree)
extern void DM_cmd_the_Lowest_Highest_Limit_Voltage(int num, float low_v, float high_v);//動作範囲電圧  low[V] , high[V]
extern void	DM_cmd_Operating_Mode(int num, DM_OPERATING_MODE mode);//Drive Direction設定
extern void	DM_cmd_Drive_Mode(int num, DM_DRIVE_MODE mode);//Drive Direction設定
extern void	DM_cmd_Max_Torque(int num, int trq);//最大トルク設定
extern void	DM_cmd_Max_PWM(int num, float pwm); // From 0.0 to 1.0
extern void	DM_cmd_Max_Acceleration(int num, float accel);// From 0 to 32767 * 214.577[rev/min2]
extern void	DM_cmd_Max_Velocity(int num, float vel);// From 0rpm to 1023*0.229rpm
extern void	DM_cmd_Status_Level(int num, BYTE level);//Status Packetのreturn level設定
extern void	DM_cmd_Alarm_LED(int num, int ebits);//アラームLED
extern void	DM_cmd_Shutdown(int num, DM_SHUTDOWN ebits); // 非常停止
extern void	DM_cmd_LED(int num, BYTE on_off);//LEDの設定
extern void	DM_cmd_Reset(int num);//RESET
extern void	DM_cmd_Goal_Position(int num, LWORD ang);//移動目標位置
extern void	DM_cmd_Goal_Velocity(int num, float rpm);//移動速度 (from -1023*0.229rpm to 1023*0.229rpm)
#define DM_cmd_Moving_Speed(N,S) DM_cmd_Goal_Velocity(N,S*0.229)
extern LWORD DM_cmd_Present_Position(int num);//現在位置(READING)
extern LWORD DM_cmd_Present_Velocity(int num);//現在速度(read)
extern WORD DM_cmd_Present_Voltage(int num);//印加電圧(read)
extern BYTE	DM_cmd_Present_Temperature(int num);//内部摂氏温度(read)
extern WORD	DM_cmd_Present_Load(int num);//現在駆動している負荷の大きさ(read)
extern WORD	DM_cmd_Present_PWM(int num);//現在PWMの大きさ(read)
extern BYTE	DM_cmd_Registered_Instruction(int num);//ＲＥＧとＡＣＴＩＯＮの関係
extern BYTE	DM_cmd_Moving(int num);//自力で動いているか調べる(read)
extern void	DM_cmd_Lock(int num);//ロック
extern void	DM_cmd_Margin(int num, int CW_Margin, int CCW_Margin);//Compliance Margin
extern void	DM_cmd_Slope(int num, int CW_Slope, int CCW_Slope);//Compliance Slope
extern void	DM_cmd_Punch(int num, int punch);//モータへの最小限の電流量
extern void	DM_cmd_ping(int num);//Status Packetをreturnさせる
extern void	DM_cmd_Torque_Enable(int num, int on_off);//Motor Torque ON/OFF
extern void	DM_cmd_Set_Number(int num);// Set Motor ID !!!Every motor will be changed!!!
extern void	DM_cmd_Set_Baudrate_3000000(void); // !!!Every motor will be changed!!!
extern void	DM_cmd_Set_Baudrate_57600(void); // !!!Every motor will be changed!!!

extern void	DM_Power_Off_All_Servo(void);
extern void	DM_Power_On_All_Servo(void);
extern void	DM_set_angle(int num, float angle);
extern void	DM_set_limit_angle(int num, float cw_limit, float ccw_limit);
extern void DM_set_velocity(int num, float rpm);// rpm[rpm]
extern void	DM_Wheel_Mode(int num);
#define DM_Endless_Revolution_Mode DM_Wheel_Mode
extern void DM_Multiturn_Mode(int num);
extern void	DM_Servo_Mode(int num);
extern float DM_get_angle(int num); // (degree)
extern float DM_get_velocity(int num); // (rpm)
extern float DM_get_temperature(int num); //内部摂氏温度(degree)
extern float DM_get_load(int num);// Parcent
extern float DM_get_voltage(int num);//V
extern void	DM_Wait_Done(int num);

#define	TORQUE_LOW	0xff
#define	TORQUE_MID	0x1ff
#define	TORQUE_HIGH	0x2ff

#define	SPEED_LOW	0x8
#define	SPEED_MID	0x10
#define	SPEED_HIGH	0x20

/************************************/
/*   Sample of set_angle for servo  */
/************************************/

#ifdef SAMPLE___

/* Each Servo Signs of rolling direction and Offset Angle */

/**********************
 Servo Number   sign
     and          of
   location    direction

		0    3      -   +
		1    4      +   -
		2 13 5      -   +
		| 12 |      | A |
		|    |      |   |
		8   11      +   -
		7   10      -   +
		6    9      +   -
		
**********************/

DM_ServoInfo	ServoInfo[] = { 
	/*SRV 0*/{1 , -1,  90, DM_KIND_RX}, /*SRV 1*/{3 , +1,  85, DM_KIND_RX} };

#ifndef numof
#define numof(X) (sizeof(X)/sizeof(X[0]))
#endif

int main(void)
{
	float tmp_angle;
	DM_Init_Dynamixel(numof(ServoInfo), ServoInfo);
	DM_Power_Off_All_Servo(void);
	DM_set_angle(0, 60.0); // SRV 0
	tmp_angle = DM_get_angle(0);
	DM_Wait_Done(0);
	DM_Destroy_RS485();
}

#endif

#endif
