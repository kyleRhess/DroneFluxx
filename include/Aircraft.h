#ifndef AIRCRAFT_H_ /* include guard */
#define AIRCRAFT_H_

#include <stdio.h>

typedef int bool;
#define true 1
#define false 0

#define MOTOR_ESC_1 	TIM_CHANNEL_1 //e3
#define MOTOR_ESC_2 	TIM_CHANNEL_2
#define MOTOR_ESC_3 	TIM_CHANNEL_3 //e4
#define MOTOR_ESC_4 	TIM_CHANNEL_4

#define CHANNEL_1_ROLL 	0
#define CHANNEL_2_PITCH 1
#define CHANNEL_3_YAW 	2
#define CHANNEL_4_THOT 	3
#define CHANNEL_NUM 	4

#define SAMPLE_RATE_HZ(xxx) (SAMPLE_RATE / xxx)

typedef enum
{
	LED_A = 0,
	LED_B = 1,
	LED_C = 2,
	LED_D = 3
} CRAFT_LED;

typedef enum
{
	AIRCRAFT_STATE_NONE = -1,
	AIRCRAFT_STATE_INIT,
	AIRCRAFT_STATE_TX_CAL,
	AIRCRAFT_STATE_IDLE,
	AIRCRAFT_STATE_ALIGNING,
	AIRCRAFT_STATE_ARMED,
	AIRCRAFT_STATE_DISARMED
} CRAFT_STATE;

extern volatile CRAFT_STATE AIRCRAFT_STATE;

extern float alt_Ahrs;
extern float roll_Ahrs;
extern float pitch_Ahrs;
extern float yaw_Ahrs;
extern float xgyro_Ahrs;
extern float ygyro_Ahrs;
extern float zgyro_Ahrs;
extern float xaccl_Ahrs;
extern float yaccl_Ahrs;
extern float zaccl_Ahrs;
extern float roll_Input;
extern float pitch_Input;
extern float yaw_Input;
extern float throttle_Input;
extern float alpha_Yaw;
extern float throttleBias;
extern float yawBias;
extern float pitchBias;
extern float rollBias;

extern float delta_alt_Ahrs;
extern float delta_roll_Ahrs;
extern float delta_pitch_Ahrs;
extern float delta_yaw_Ahrs;
extern float last_alt_Ahrs;
extern float last_roll_Ahrs;
extern float last_pitch_Ahrs;
extern float last_yaw_Ahrs;

void aircraft_GetRxInput(void);
bool aircraft_CalibratingInput(void);
bool aircraft_IsCalibrated(void);
bool aircraft_IsFlying(void);
bool aircraft_Disarming(void);
bool aircraft_Arming(void);
void aircraft_UpdateMotors(void);
void aircraft_WriteLED(int,int);
void aircraft_FlashLED(int,uint16_t);
void aircraft_Reset(void);

#endif
