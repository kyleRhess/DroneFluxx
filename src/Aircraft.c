#include "Aircraft.h"
#include "System.h"

/*
 * Exponent terms for rate control.
 * Not used yet. Will be used in: aircraft_GetRxInput()
 */
#define R_EX_3 0.0005f
#define R_EX_2 0.0f
#define R_EX_1 0.6667f
#define R_EX_0 0.0f

#define MIN_IDLE_THROT 5.0f
#define MAX_CONT_THROT 80.0f
#define MIN_IDLE_ALT 0.0f
#define MAX_CONT_ALT 6.0f

#define MOTOR_FILT_K 0.45045f

float xgyro_Ahrs, ygyro_Ahrs, zgyro_Ahrs = 0.0f;
float xaccl_Ahrs, yaccl_Ahrs, zaccl_Ahrs = 0.0f;
float alt_Ahrs, roll_Ahrs, pitch_Ahrs, yaw_Ahrs = 0.0f;
float roll_Input, pitch_Input, yaw_Input = 0.0f;
float throt_Input = 0.0f;
float alpha_Yaw = 0.0f; //Initial yaw value
float throttleBias, yawBias, pitchBias, rollBias = 0.0f;

float channel_Max[4] = {-100000.0f, -100000.0f, -100000.0f, -100000.0f };
float channel_Min[4] = { 100000.0f,  100000.0f,  100000.0f,  100000.0f };

uint8_t flashFreq[4] = {0, 0, 0, 0};

/*
 * RC Receiver is updated by timer interrupt and the results are gathered here
 */
void aircraft_GetRxInput()
{
	// Map all microsecond inputs to % power or degrees
	roll_Input  = RATE_SCALE * mapVal(	channelPulseWidth_us[CHANNEL_1_ROLL],
										channel_Min[CHANNEL_1_ROLL],
										channel_Max[CHANNEL_1_ROLL], -20.0f, 20.0f);

	pitch_Input = RATE_SCALE * mapVal(	channelPulseWidth_us[CHANNEL_2_PITCH],
										channel_Min[CHANNEL_2_PITCH],
										channel_Max[CHANNEL_2_PITCH], -20.0f, 20.0f);

	yaw_Input  	= RATE_SCALE * mapVal(	channelPulseWidth_us[CHANNEL_3_YAW],
										channel_Min[CHANNEL_3_YAW],
										channel_Max[CHANNEL_3_YAW], -90.0f,  90.0f);

	throt_Input = 			   mapVal(	channelPulseWidth_us[CHANNEL_4_THOT],
										channel_Min[CHANNEL_4_THOT],
										channel_Max[CHANNEL_4_THOT],
										MIN_IDLE_THROT,  MAX_CONT_THROT); // (%) Limit top-end so controller can work

	roll_Input 	= 0.0018f*(roll_Input*roll_Input*roll_Input) 	+ 1.7708f*(roll_Input);		// 3rd order exponential
	pitch_Input = 0.0018f*(pitch_Input*pitch_Input*pitch_Input) + 1.7708f*(pitch_Input);	// 3rd order exponential

	if(AIRCRAFT_STATE >= AIRCRAFT_STATE_IDLE)
	{
		throt_Input 	-= (throttleBias - MIN_IDLE_THROT);
		yaw_Input 		-= yawBias;
		pitch_Input 	-= pitchBias;
		roll_Input 		-= rollBias;
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

/*
 * Return if RC receiver has been calibrated
 */
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
	if(throt_Input > MIN_THROT) rc = true;
	return rc;
}

/*
 * Throttle stick to the bottom-left:
 * This triggers aircraft disarming.
 */
bool aircraft_Disarming()
{
	bool rc = false;
	if(throt_Input < MIN_THROT && yaw_Input > 8.5f*RATE_SCALE)
	{
		if(fabsf(roll_Input) < 5.0f*RATE_SCALE && fabsf(pitch_Input) < 5.0f*RATE_SCALE)
			rc = true;
	}
	return rc;
}

/*
 * Throttle stick to the bottom-right:
 * This triggers aircraft arming.
 */
bool aircraft_Arming()
{
	bool rc = false;
	if(throt_Input < MIN_THROT && yaw_Input < -8.5f*RATE_SCALE)
	{
		if((roll_Input < -19.0f*RATE_SCALE) && (pitch_Input > 19.0f*RATE_SCALE))
		{
			// FLAG FOR RATE CONTROL
			rc = true;

			// Reset PID terms
			proportionalGain[0] 		= 0.210000f;
			proportionalGain[1] 		= 0.210000f;
			proportionalGain[2] 		= 0.200000f;
			proportionalGain[3] 		= 2.20f;

			integralGain[0] 			= 0.212125f;
			integralGain[1] 			= 0.212125f;
			integralGain[2] 			= 0.16125f;
			integralGain[3] 			= 2.50f;

			derivativeGain[0] 			= 0.00010f;
			derivativeGain[1] 			= 0.00010f;
			derivativeGain[2] 			= 0.000010f;
			derivativeGain[3] 			= 0.1750f;

			proportionalGain[PID_ROLL] 	*= 0.80f;
			integralGain[PID_ROLL] 		*= 0.80f;
			derivativeGain[PID_ROLL] 	*= 0.80f;

			flightControl[PID_XGYR].kP 	=  proportionalGain[PID_XGYR] * 0.75f;
			flightControl[PID_YGYR].kP 	=  proportionalGain[PID_YGYR] * 0.75f;
			flightControl[PID_ZGYR].kP 	=  proportionalGain[PID_ZGYR] * 0.85f;
		}
	}
	return rc;
}

/*
 * Update all motor values with PID signal outputs
 */
void aircraft_UpdateMotors()
{
	static float newMotorVal[4] = {0.0f};

	motorPower[MOTOR_FRONT_LEFT]  = throt_Input;
	motorPower[MOTOR_FRONT_RIGHT]  = throt_Input;
	motorPower[MOTOR_BACK_LEFT]  = throt_Input;
	motorPower[MOTOR_BACK_RIGHT]  = throt_Input;

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

#ifdef MOTOR_FILT_K
	/*
	 * Filter PID motor signals to prevent ESC confusion
	 * Not ideal. May need to reduce/remove filtering
	 */
	for (int i = 0; i < 4; ++i)
	{
		newMotorVal[i] = (1.0f - MOTOR_FILT_K)*newMotorVal[i] + (MOTOR_FILT_K * motorPower[i]);
	}
#endif

	// Scale power (0->100) to a pulse width (125->250)
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, mapVal(newMotorVal[MOTOR_FRONT_LEFT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, mapVal(newMotorVal[MOTOR_FRONT_RIGHT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, mapVal(newMotorVal[MOTOR_BACK_LEFT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
	PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(newMotorVal[MOTOR_BACK_RIGHT], 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
}

/*
 *  Control an LED
 */
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

/*
 *  Control an LED
 */
void aircraft_FlashLED(int LED, uint16_t Freq)
{
	flashFreq[LED] = (uint8_t)Freq;

//	static uint32_t lastFlashTicks[4] = {0,0,0,0};
//	if((TotalReadTicks - lastFlashTicks[LED]) > SAMPLE_RATE_HZ(Freq/2))
//	{
//		lastFlashTicks[LED] = TotalReadTicks;
//		switch(LED)
//		{
//		case LED_A:
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
//			break;
//		case LED_B:
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			break;
//		case LED_C:
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
//			break;
//		case LED_D:
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
//			break;
//		default:
//			break;
//		}
//	}
}

/*
 * Completely resets all parameters.
 * Should only be called while not in flight.
 */
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
	throt_Input = 0.0f;
	alpha_Yaw = 0.0f; // Initial yaw value

	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_FRONT_LEFT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_FRONT_RIGHT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_BACK_LEFT], 0.0f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, motorPower[MOTOR_BACK_RIGHT], 0.0f);
}
