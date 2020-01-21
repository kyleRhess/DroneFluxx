#include "Aircraft.h"
#include "System.h"

/*
 * Exponent terms for rate control
 */
#define R_EX_3 0.0005f
#define R_EX_2 0.0f
#define R_EX_1 0.6667f
#define R_EX_0 0.0f

#define MIN_IDLE_THROT 5.0f
#define MAX_CONT_THROT 80.0f
#define MIN_IDLE_ALT 0.0f
#define MAX_CONT_ALT 6.0f

float last_alt_Ahrs, last_roll_Ahrs, last_pitch_Ahrs, last_yaw_Ahrs = 0.0f;
float delta_alt_Ahrs, delta_roll_Ahrs, delta_pitch_Ahrs, delta_yaw_Ahrs = 0.0f;

float xgyro_Ahrs, ygyro_Ahrs, zgyro_Ahrs = 0.0f;
float alt_Ahrs, roll_Ahrs, pitch_Ahrs, yaw_Ahrs = 0.0f;
float roll_Input, pitch_Input, yaw_Input = 0.0f;
float throttle_Input = 0.0f;
float alpha_Yaw = 0.0f; //Initial yaw value
float throttleBias, yawBias, pitchBias, rollBias = 0.0f;

bool aircraft_AttControl = false;

float channel_Max[4] = {-100000.0f, -100000.0f, -100000.0f, -100000.0f };
float channel_Min[4] = { 100000.0f,  100000.0f,  100000.0f,  100000.0f };

void aircraft_GetRxInput()
{
	// Map all microsecond inputs to % power or degrees
	if(aircraft_AttControl && AIRCRAFT_STATE >= AIRCRAFT_STATE_ARMED)
	{
		roll_Input  	 = ATT_SCALE * mapVal((channelPulseWidth_us[CHANNEL_1_ROLL]),  	channel_Min[CHANNEL_1_ROLL], channel_Max[CHANNEL_1_ROLL], -20.0f, 20.0f); // (°/s)
		pitch_Input 	 = ATT_SCALE * mapVal((channelPulseWidth_us[CHANNEL_2_PITCH]), 	channel_Min[CHANNEL_2_PITCH], channel_Max[CHANNEL_2_PITCH], -20.0f, 20.0f); // (°/s)
		yaw_Input  		 = ATT_SCALE * mapVal((channelPulseWidth_us[CHANNEL_3_YAW]),   channel_Min[CHANNEL_3_YAW], channel_Max[CHANNEL_3_YAW], -90.0f,  90.0f); // (°/s)
		throttle_Input   = mapVal((channelPulseWidth_us[CHANNEL_4_THOT]), 	channel_Min[CHANNEL_4_THOT], channel_Max[CHANNEL_4_THOT],  MIN_IDLE_ALT,  MAX_CONT_ALT); // (%) Limit top-end so controller can work

		if(AIRCRAFT_STATE >= AIRCRAFT_STATE_IDLE)
		{
			throttle_Input 	-= (throttleBias - MIN_IDLE_ALT - MIN_IDLE_THROT);
			yaw_Input 		-= yawBias;
			pitch_Input 	-= pitchBias;
			roll_Input 		-= rollBias;
		}
	}
	else
	{
		roll_Input  	 = RATE_SCALE * mapVal((channelPulseWidth_us[CHANNEL_1_ROLL]),  	channel_Min[CHANNEL_1_ROLL], channel_Max[CHANNEL_1_ROLL], -20.0f, 20.0f); // (°/s)
		pitch_Input 	 = RATE_SCALE * mapVal((channelPulseWidth_us[CHANNEL_2_PITCH]), 	channel_Min[CHANNEL_2_PITCH], channel_Max[CHANNEL_2_PITCH], -20.0f, 20.0f); // (°/s)
		yaw_Input  		 = RATE_SCALE * mapVal((channelPulseWidth_us[CHANNEL_3_YAW]),   channel_Min[CHANNEL_3_YAW], channel_Max[CHANNEL_3_YAW], -90.0f,  90.0f); // (°/s)
		throttle_Input   = mapVal((channelPulseWidth_us[CHANNEL_4_THOT]), 	channel_Min[CHANNEL_4_THOT], channel_Max[CHANNEL_4_THOT],  MIN_IDLE_THROT,  MAX_CONT_THROT); // (%) Limit top-end so controller can work

		roll_Input = 0.0018f*(roll_Input*roll_Input*roll_Input) + 1.7708f*(roll_Input);			// 3rd order exponential
		pitch_Input = 0.0018f*(pitch_Input*pitch_Input*pitch_Input) + 1.7708f*(pitch_Input);	// 3rd order exponential

		if(AIRCRAFT_STATE >= AIRCRAFT_STATE_IDLE)
		{
			throttle_Input 	-= (throttleBias - MIN_IDLE_THROT);
			yaw_Input 		-= yawBias;
			pitch_Input 	-= pitchBias;
			roll_Input 		-= rollBias;
		}
	}
}

/*
 * Gets the max/min values from the RC Tx since these aren't exact
 * Returns false when measured max/min inputs stop updating
 */
bool aircraft_CalibratingInput()
{
	bool stillChanging = false;
	for (int i = 0; i < 4; ++i)
	{
		if(channelPulseWidth_us[i] < channel_Min[i])
		{
			channel_Min[i] = channelPulseWidth_us[i];
			stillChanging = true;
		}

		if(channelPulseWidth_us[i] > channel_Max[i])
		{
			channel_Max[i] = channelPulseWidth_us[i];
			stillChanging = true;
		}
	}

	return stillChanging;
}

bool aircraft_IsCalibrated()
{
	bool rc = false;

	if(channel_Min[CHANNEL_1_ROLL] < 1400.0f && channel_Max[CHANNEL_1_ROLL] > 1800.0f)
	{
		if(channel_Min[CHANNEL_2_PITCH] < 1400.0f && channel_Max[CHANNEL_2_PITCH] > 1800.0f)
		{
			if(channel_Min[CHANNEL_3_YAW] < 1400.0f && channel_Max[CHANNEL_3_YAW] > 1800.0f)
			{
				if(channel_Min[CHANNEL_4_THOT] < 1400.0f && channel_Max[CHANNEL_4_THOT] > 1800.0f)
				{
					rc = true;
				}
			}
		}
	}

	return rc;
}


/*
 * Return flight status
 */
bool aircraft_IsFlying()
{
	bool rc = false;

	// Only return true if high throttle
	if((throttle_Input > MIN_THROT) && !aircraft_AttControl) rc = true;
	else if((throttle_Input > 0.1f) && aircraft_AttControl) rc = true;
	return rc;
}

// Throttle stick to the bottom-left
bool aircraft_Disarming()
{
	bool rc = false;

	if(aircraft_AttControl)
	{
		if(throttle_Input < 0.1f && yaw_Input > 8.5f*ATT_SCALE)
		{
			if(fabsf(roll_Input) < 5.0f*ATT_SCALE && fabsf(pitch_Input) < 5.0f*ATT_SCALE) rc = true;
		}
	}
	else
	{
		if(throttle_Input < MIN_THROT && yaw_Input > 8.5f*RATE_SCALE)
		{
			if(fabsf(roll_Input) < 5.0f*RATE_SCALE && fabsf(pitch_Input) < 5.0f*RATE_SCALE) rc = true;
		}
	}
	return rc;
}

// Throttle stick to the bottom-right
bool aircraft_Arming()
{
	bool rc = false;
	if(throttle_Input < MIN_THROT && yaw_Input < -8.5f*RATE_SCALE)
	{
		if((roll_Input < -19.0f*RATE_SCALE) && (pitch_Input > 19.0f*RATE_SCALE))
		{
			/*
			 * FLAG FOR RATE CONTROL
			 */
			rc = true;
			aircraft_AttControl = false;


			/*
			 * Reset PID terms
			 */
			proportionalGain[0] 	= 0.210000f;
			proportionalGain[1] 	= 0.210000f;
			proportionalGain[2] 	= 0.200000f;
			proportionalGain[3] 	= 2.20f;

			integralGain[0] 	= 0.212125f;
			integralGain[1] 	= 0.212125f;
			integralGain[2] 	= 0.16125f;
			integralGain[3] 	= 2.50f;

			derivativeGain[0] 	= 0.00010f;
			derivativeGain[1] 	= 0.00010f;
			derivativeGain[2] 	= 0.000010f;
			derivativeGain[3] 	= 0.1750f;

			proportionalGain[PID_ROLL] *= 0.80f;
			integralGain[PID_ROLL] *= 0.80f;
			derivativeGain[PID_ROLL] *= 0.80f;

			flightControl[PID_XGYR].kP =  proportionalGain[PID_XGYR] * 0.75f;
			flightControl[PID_YGYR].kP =  proportionalGain[PID_YGYR] * 0.75f;
			flightControl[PID_ZGYR].kP =  proportionalGain[PID_ZGYR] * 0.85f;
		}
		else if((roll_Input > 19.0f*RATE_SCALE) && (pitch_Input > 19.0f*RATE_SCALE))
		{
			/*
			 * FLAG FOR ATTITUDE CONTROL
			 */
			rc = true;
			aircraft_AttControl = true;


			/*
			 * Reset PID terms
			 */
			proportionalGain[PID_PITCH] 	= 0.20000f;
			proportionalGain[PID_ROLL] 		= 0.20000f;
			proportionalGain[PID_YAW] 		= 0.20000f;
			proportionalGain[PID_ALT] 		= 0.20000f;

			integralGain[PID_PITCH] 		= 0.0000f;
			integralGain[PID_ROLL] 			= 0.0000f;
			integralGain[PID_YAW] 			= 0.0000f;
			integralGain[PID_ALT] 			= 0.0000f;

			derivativeGain[PID_PITCH]	 	= 0.00f;
			derivativeGain[PID_ROLL] 		= 0.00f;
			derivativeGain[PID_YAW] 		= 0.00f;
			derivativeGain[PID_ALT] 		= 0.00f;

		}
	}
	return rc;
}

static float newMotorVal[4] = {0.0f};

// Update all motor values with PID outputs
void aircraft_UpdateMotors()
{
	if(1)//(!aircraft_AttControl)
	{
		motorPower[MOTOR_FRONT_LEFT]  = throttle_Input;
		motorPower[MOTOR_FRONT_RIGHT]  = throttle_Input;
		motorPower[MOTOR_BACK_LEFT]  = throttle_Input;
		motorPower[MOTOR_BACK_RIGHT]  = throttle_Input;
	}

	motorPower[MOTOR_FRONT_LEFT] += PID_GetOutput(&flightControl[PID_ALT]);
	motorPower[MOTOR_FRONT_LEFT] += PID_GetOutput(&flightControl[PID_PITCH]);
	motorPower[MOTOR_FRONT_LEFT] += PID_GetOutput(&flightControl[PID_ROLL]);
	motorPower[MOTOR_FRONT_LEFT] += PID_GetOutput(&flightControl[PID_YAW]);

	motorPower[MOTOR_FRONT_RIGHT] += PID_GetOutput(&flightControl[PID_ALT]);
	motorPower[MOTOR_FRONT_RIGHT] += PID_GetOutput(&flightControl[PID_PITCH]);
	motorPower[MOTOR_FRONT_RIGHT] -= PID_GetOutput(&flightControl[PID_ROLL]);
	motorPower[MOTOR_FRONT_RIGHT] -= PID_GetOutput(&flightControl[PID_YAW]);

	motorPower[MOTOR_BACK_LEFT] += PID_GetOutput(&flightControl[PID_ALT]);
	motorPower[MOTOR_BACK_LEFT] -= PID_GetOutput(&flightControl[PID_PITCH]);
	motorPower[MOTOR_BACK_LEFT] += PID_GetOutput(&flightControl[PID_ROLL]);
	motorPower[MOTOR_BACK_LEFT] -= PID_GetOutput(&flightControl[PID_YAW]);

	motorPower[MOTOR_BACK_RIGHT] += PID_GetOutput(&flightControl[PID_ALT]);
	motorPower[MOTOR_BACK_RIGHT] -= PID_GetOutput(&flightControl[PID_PITCH]);
	motorPower[MOTOR_BACK_RIGHT] -= PID_GetOutput(&flightControl[PID_ROLL]);
	motorPower[MOTOR_BACK_RIGHT] += PID_GetOutput(&flightControl[PID_YAW]);

#define K_MOT 0.45045f
#ifdef K_MOT
	for (int i = 0; i < 4; ++i)
	{
		newMotorVal[i] = (1.0f - K_MOT)*newMotorVal[i] + (K_MOT * motorPower[i]);
	}
#endif

	// Scale power (0->100) to a pulse width (125->250)
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, mapVal(newMotorVal[MOTOR_FRONT_LEFT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, mapVal(newMotorVal[MOTOR_FRONT_RIGHT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, mapVal(newMotorVal[MOTOR_BACK_LEFT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(newMotorVal[MOTOR_BACK_RIGHT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
}

// Control an LED
void aircraft_WriteLED(int LED, int state)
{
	switch(LED)
	{
	case LED_A:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, state);
		break;
	case LED_B:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state);
		break;
	case LED_C:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, state);
		break;
	case LED_D:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, state);
		break;
	default:
		break;
	}
}


// Control an LED
static uint32_t lastFlashTicks[4] = {0,0,0,0};
void aircraft_FlashLED(int LED, uint16_t Freq)
{
	if((TotalReadTicks - lastFlashTicks[LED]) > SAMPLE_RATE_HZ(Freq/2))
	{
		lastFlashTicks[LED] = TotalReadTicks;
		switch(LED)
		{
		case LED_A:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
			break;
		case LED_B:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			break;
		case LED_C:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
			break;
		case LED_D:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			break;
		default:
			break;
		}
	}
}


void aircraft_Reset()
{
	aircraft_WriteLED(LED_A, 0);
	aircraft_WriteLED(LED_B, 0);
	aircraft_WriteLED(LED_C, 0);
	aircraft_WriteLED(LED_D, 0);

	for (int i = 0; i < 4; ++i)
	{
		channel_Max[i] = -100000.0f;
		channel_Min[i] =  100000.0f;
	}

	alt_Ahrs = roll_Ahrs = pitch_Ahrs = yaw_Ahrs = 0.0f;
	xgyro_Ahrs = ygyro_Ahrs = zgyro_Ahrs = 0.0f;
	roll_Input = pitch_Input = yaw_Input = 0.0f;
	throttle_Input = 0.0f;
	alpha_Yaw = 0.0f; //Initial yaw value

	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_FRONT_LEFT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_FRONT_RIGHT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_BACK_LEFT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_BACK_RIGHT], 0.0f);
}
