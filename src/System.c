#include "System.h"



static int status = HAL_OK;

/*
 * Extern variables used throughout
 */
float channelPulseWidth_us[CHANNEL_NUM];
PWM_Out PWMtimer;
PID_Controller flightControl[PID_NUM];
float motorPower[MOTOR_NUM];
float zacc, xacc, yacc = 0.0f;
float zgyr, xgyr, ygyr = 0.0f;
float zgyrBias, xgyrBias, ygyrBias, altBias = 0.0f;
float xmag, ymag, zmag, tempC = 0.0f;
float altitude = 0.0f;
double seaLevelPress = -99.0;

float lat, lon, heading, groundSpeed, numSVs = 0.0f;

bool system_Aligned = false;

matrix3x3 LevelDCM;
matrix3x3 rDCM;
vector3 rVect;


float proportionalGain[PID_NUM] 	= {0.210000f, 0.210000f, 0.200000f, 2.20f};
float integralGain[PID_NUM] 		= {0.212125f, 0.212125f, 0.16125f, 2.50f};
float derivativeGain[PID_NUM] 		= {0.00010f, 0.00010f, 0.000010f, 0.175f};


volatile SYS_STATE SYSTEM_STATE = SYS_STATE_NONE;
volatile CRAFT_STATE AIRCRAFT_STATE = AIRCRAFT_STATE_NONE;


void InitializeSystem()
{
	// Less inertia on roll axis so reduce gains
	proportionalGain[PID_ROLL] *= 0.80f;
	integralGain[PID_ROLL] *= 0.80f;
	derivativeGain[PID_ROLL] *= 0.80f;

	status = HAL_Init();
	status = initData();
	status = InitSamplingTimer(); // Sensor sampling timer
	status = InitReceiverTimer(); // Receiver input timer
	status = InitPWMOutput();  	  // ESC output PWM
	status = InitPID();			  // PID controller setup
	status = Reset_Init();
	InitSPIBus();
	status = InitSerial(115200, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);		  // Serial comm. setup

	flightControl[PID_XGYR].kP =  proportionalGain[PID_XGYR] * 0.75f;
	flightControl[PID_YGYR].kP =  proportionalGain[PID_YGYR] * 0.75f;
	flightControl[PID_ZGYR].kP =  proportionalGain[PID_ZGYR] * 0.85f;

	// Catch init failure
	while(status){;}



	/*
	 * Start data acquisition sample timer interrupt
	 */
	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn,0,0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);



	// Set motor PWM to 0%
	for (size_t i = 0; i < sizeof(motorPower)/sizeof(motorPower[0]); ++i)
	{
		motorPower[i] = 0.0f;
	}
}


void RunSystem()
{
	switch(SYSTEM_STATE)
	{
		case SYS_STATE_NONE:
			break;
		case SYS_STATE_IDLE:
			break;
		case SYS_STATE_PROCESSING:
			procRawData();
			break;
		case SYS_STATE_CONTROLLING:
			runController();
			break;
		case SYS_STATE_RESET:
			resetController();
			break;
	}
}

// This gets called at CONTROL_RATE (Hz).
void runController()
{
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	static float xgyrBiasTemp, ygyrBiasTemp, zgyrBiasTemp, altBiasTemp = 0.0f;
	static int armingCount = 0;
	static long gyroBiasCount = 0;

	switch(AIRCRAFT_STATE)
	{
		case AIRCRAFT_STATE_NONE:
			break;
		case AIRCRAFT_STATE_ALIGNING:

			// Check for a restart while in flight
			if(aircraft_IsFlying() && (xgyro_Ahrs > 10.0f || ygyro_Ahrs > 10.0f || zgyro_Ahrs > 10.0f))
			{
				//Skip alignment
				xgyrBiasTemp = 0;
				ygyrBiasTemp = 0;
				zgyrBiasTemp = 0;
				altBiasTemp = 0;
				beta = 0.08f;
				system_Aligned = true;
				AIRCRAFT_STATE = AIRCRAFT_STATE_ARMED;
			}

			if(SYS_READ_TIME(TotalReadTicks) < 5.0f)
			{
				aircraft_FlashLED(LED_A, 2);
			}
			else
			{
				if((SYS_READ_TIME(TotalReadTicks) < 10.0f) && !system_Aligned)
				{
					aircraft_FlashLED(LED_A, 4);

					//Get sensor biases to subtract for first X seconds of initialization
					xgyrBiasTemp += xgyr;
					ygyrBiasTemp += ygyr;
					zgyrBiasTemp += zgyr;
					altBiasTemp += altitude;

					//Set filter to converge fast at start
					beta = 5.5f;

					gyroBiasCount++;
				}
				else if(!system_Aligned)
				{
					xgyrBiasTemp /= (float)gyroBiasCount;
					ygyrBiasTemp /= (float)gyroBiasCount;
					zgyrBiasTemp /= (float)gyroBiasCount;
					altBiasTemp /= (float)gyroBiasCount;

					xgyrBias = xgyrBiasTemp;
					ygyrBias = ygyrBiasTemp;
					zgyrBias = zgyrBiasTemp;
					altBias = altBiasTemp;

					alpha_Yaw = yaw_Madgwick;

					//Set filter back to normal
					beta = 0.16f;

					PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
					PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
					PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
					PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));

					system_Aligned = true;
					AIRCRAFT_STATE = AIRCRAFT_STATE_INIT;
				}
			}
			break;
		case AIRCRAFT_STATE_INIT:
			// Light LED_A
			aircraft_FlashLED(LED_A, 8);

			static int pwmPulse = 0;


			pwmPulse++;
			if(pwmPulse == 50)
			{
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, 30.0f);
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, 30.0f);
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, 30.0f);
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, 30.0f);

				pwmPulse = 9999;
			}
			else if(pwmPulse == 10000)
			{

				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
				PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
				pwmPulse = 0;
			}


			//PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));


			// Wait for RC Tx to turn on
			if(channelPulseWidth_us[CHANNEL_1_ROLL] > 0.0f)
			{
				armingCount++;

				if(armingCount >= 10)
				{
					armingCount = 0;
					aircraft_WriteLED(LED_A, 1);
					AIRCRAFT_STATE = AIRCRAFT_STATE_TX_CAL;
				}
			}
			break;
		case AIRCRAFT_STATE_TX_CAL:
			// Light LED_B
			aircraft_FlashLED(LED_B, 4);

			// Check for calibration complete signal
			// Move sticks to extremes during this time.
			if(!aircraft_CalibratingInput() && aircraft_IsCalibrated())
			{
				aircraft_GetRxInput();
				aircraft_WriteLED(LED_C, 1);
				armingCount++;
				if(armingCount >= 2000)
				{
					throttleBias 	= throttle_Input;
					yawBias 		= yaw_Input;
					pitchBias 		= pitch_Input;
					rollBias 		= roll_Input;

					aircraft_WriteLED(LED_C, 0);
					armingCount = 0;
					AIRCRAFT_STATE = AIRCRAFT_STATE_IDLE;
				}
			}
			else
			{
				aircraft_WriteLED(LED_C, 0);
				armingCount = 0;
			}

			break;
		case AIRCRAFT_STATE_IDLE:
			// Light LED_B
			aircraft_FlashLED(LED_A, 1);
			aircraft_FlashLED(LED_B, 1);
			aircraft_FlashLED(LED_C, 1);
			aircraft_FlashLED(LED_D, 1);

			aircraft_GetRxInput();

			/*
			 *
			 * Arming signal
			 *
			 */
			if(aircraft_Arming() || (armingCount >= 100))
			{
				armingCount++;
				if(armingCount >= 100)
				{
					// Must move sticks back to 0 before arming complete
					if(throttle_Input < MIN_THROT && (fabsf(yaw_Input) < 2.0f * RATE_SCALE) && (fabsf(roll_Input) < 3.0f * RATE_SCALE) && (fabsf(pitch_Input) < 3.0f * RATE_SCALE))
					{
						aircraft_WriteLED(LED_B, 0);
						aircraft_WriteLED(LED_C, 0);
						aircraft_FlashLED(LED_A, 0);

						armingCount = 0;
						AIRCRAFT_STATE = AIRCRAFT_STATE_ARMED;
					}
				}
			}
			else
			{
				armingCount = 0;
			}

			break;
		case AIRCRAFT_STATE_ARMED:
			aircraft_GetRxInput();

			if(throttle_Input > 80.0f)
				aircraft_WriteLED(LED_D, 1);
			else
				aircraft_WriteLED(LED_D, 0);

			if(throttle_Input > 60.0f)
				aircraft_WriteLED(LED_C, 1);
			else
				aircraft_WriteLED(LED_C, 0);

			if(throttle_Input > 40.0f)
				aircraft_WriteLED(LED_B, 1);
			else
				aircraft_WriteLED(LED_B, 0);

			if(throttle_Input > 20.0f)
				aircraft_WriteLED(LED_A, 1);
			else
				aircraft_WriteLED(LED_A, 0);

			/*
			 *
			 * Disarming signal
			 *
			 */
			if(aircraft_Disarming())
			{
				armingCount++;
				if(armingCount >= 200)
				{
					armingCount = 0;
					aircraft_AttControl = false;
					AIRCRAFT_STATE = AIRCRAFT_STATE_DISARMED;
				}
			}
			else
			{
				armingCount = 0;
			}

			if(aircraft_AttControl)
			{
				/*
				 * Integrate yaw input.
				 * Deal with angle wrap around near 360°/0°
				 */
				if(alpha_Yaw + (yaw_Input / 10.0f) < 0)
				{
					alpha_Yaw += (360.0f + (yaw_Input / 10.0f));
				}
				else if (alpha_Yaw + (yaw_Input / 10.0f) > 360.0f)
				{
					alpha_Yaw += (-360.0f + (yaw_Input / 10.0f));
				}
				else
				{
					alpha_Yaw += ((yaw_Input / 10.0f));
				}

				// Apply input set-points
				PID_SetSetpoint(&flightControl[PID_ALT], throttle_Input);
				PID_SetSetpoint(&flightControl[PID_XGYR], pitch_Input);
				PID_SetSetpoint(&flightControl[PID_YGYR], roll_Input);
				PID_SetSetpoint(&flightControl[PID_ZGYR], yaw_Input);

				if(aircraft_IsFlying())
				{
					PID_Update(&flightControl[PID_XGYR], pitch_Ahrs);
					PID_Update(&flightControl[PID_YGYR], roll_Ahrs);
					PID_Update(&flightControl[PID_ZGYR], zgyro_Ahrs);
					PID_Update(&flightControl[PID_ALT], (altitude-altBias));
				}
				else
				{
					PID_Reset(&flightControl[PID_ROLL]);
					PID_Reset(&flightControl[PID_PITCH]);
					PID_Reset(&flightControl[PID_YAW]);
					PID_Reset(&flightControl[PID_ALT]);
					throttle_Input = 0.0f; // reset
					aircraft_UpdateMotors();
				}
			}
			else
			{
				/*
				 * Rate control
				 */

				// Apply input set-points
				PID_SetSetpoint(&flightControl[PID_XGYR], pitch_Input);
				PID_SetSetpoint(&flightControl[PID_YGYR], roll_Input);
				PID_SetSetpoint(&flightControl[PID_ZGYR], yaw_Input);

				// Check for loss of signal Tx
				static float lastPulseWidth[CHANNEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
				static int equalCount = 0;
				for (int channel = 0; channel < CHANNEL_NUM; ++channel)
				{
					if(lastPulseWidth[channel] == channelPulseWidth_us[channel])
						equalCount++;
					else
						equalCount = 0;

					lastPulseWidth[channel] = channelPulseWidth_us[channel];
				}

				if(aircraft_IsFlying() && equalCount < 100)
				{
//					if(throttle_Input > 40.0f)
//					{
//						float pScalar = mapVal(throttle_Input, 39.9f, 85.01f, 1.0f, 0.55f);
//
//						flightControl[PID_XGYR].kP =  proportionalGain[PID_XGYR] * pScalar;
//						flightControl[PID_YGYR].kP =  proportionalGain[PID_YGYR] * pScalar;
//						flightControl[PID_ZGYR].kP =  proportionalGain[PID_ZGYR] * pScalar;
//
//						aircraft_FlashLED(LED_D, 7);
//					}
//					else
//					{
//						flightControl[PID_XGYR].kP =  proportionalGain[PID_XGYR];
//						flightControl[PID_YGYR].kP =  proportionalGain[PID_YGYR];
//						flightControl[PID_ZGYR].kP =  proportionalGain[PID_ZGYR];
//					}

					PID_Update(&flightControl[PID_XGYR], xgyro_Ahrs);
					PID_Update(&flightControl[PID_YGYR], ygyro_Ahrs);
					PID_Update(&flightControl[PID_ZGYR], zgyro_Ahrs);
					PID_Reset(&flightControl[PID_ALT]);
				}
				else
				{
					PID_Reset(&flightControl[PID_PITCH]);
					PID_Reset(&flightControl[PID_ROLL]);
					PID_Reset(&flightControl[PID_YAW]);
					PID_Reset(&flightControl[PID_ALT]);
					throttle_Input = 0.0f; // reset since it will be idling at %20 otherwise
					aircraft_UpdateMotors();
				}
			}

			// Apply new PID outputs to ESCs
			aircraft_UpdateMotors();
			break;
		case AIRCRAFT_STATE_DISARMED:

			aircraft_WriteLED(LED_D, 0);
			aircraft_WriteLED(LED_C, 0);
			aircraft_WriteLED(LED_B, 0);

			PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_1, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
			PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_2, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
			PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_3, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));
			PWM_adjust_PulseWidth(&PWMtimer.timer, MOTOR_ESC_4, mapVal(0.0f, 	0.0f, 100.0f, MIN_ESC_US, MAX_ESC_US));

			// Zero controller otherwise
			PID_Reset(&flightControl[PID_ALT]);
			PID_Reset(&flightControl[PID_PITCH]);
			PID_Reset(&flightControl[PID_ROLL]);
			PID_Reset(&flightControl[PID_YAW]);

			armingCount++;
			if(armingCount >= 1000)
			{
				armingCount = 0;
				AIRCRAFT_STATE = AIRCRAFT_STATE_IDLE;
			}
			break;
	}

	// Check all the time for reset button pressing
	if(Reset_Check())
	{
		SYSTEM_STATE = SYS_STATE_RESET;

		xgyrBiasTemp = ygyrBiasTemp = zgyrBiasTemp = 0.0f;
		armingCount = 0;
		gyroBiasCount = 0;
	}
	else
	{
		SYSTEM_STATE = SYS_STATE_IDLE;
	}
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}

void resetController()
{
	// Zero controller
	PID_Reset(&flightControl[PID_ALT]);
	PID_Reset(&flightControl[PID_PITCH]);
	PID_Reset(&flightControl[PID_ROLL]);
	PID_Reset(&flightControl[PID_YAW]);

	aircraft_WriteLED(LED_A, 0);
	aircraft_WriteLED(LED_B, 0);
	aircraft_WriteLED(LED_C, 0);
	aircraft_WriteLED(LED_D, 0);

	aircraft_Reset();

	zacc = xacc = yacc = 0.0f;
	zgyr = xgyr = ygyr = 0.0f;
	zgyrBias = xgyrBias = ygyrBias = 0.0f;
	xgyro_Ahrs = ygyro_Ahrs = zgyro_Ahrs = 0.0f;
	zmag = xmag = ymag = tempC = 0.0f;

	system_Aligned = false;

	ReadTicks = 0;
	TotalReadTicks = 0;
	msgReadTicks = 0;

	initData();

	SYSTEM_STATE = SYS_STATE_IDLE;
	AIRCRAFT_STATE = AIRCRAFT_STATE_ALIGNING;
}

int InitPID()
{
	/*
	 * Setup PID loops for yaw_Ahrs/pitch_Ahrs/roll
	 */
	flightControl[PID_PITCH].kP = proportionalGain[PID_PITCH];
	flightControl[PID_PITCH].kI = integralGain[PID_PITCH];
	flightControl[PID_PITCH].kD = derivativeGain[PID_PITCH];
	flightControl[PID_PITCH].setPoint = 0.0f;
	flightControl[PID_PITCH].deltaTime = (1.0f / (float)CONTROL_RATE);
	PID_Initialize(&flightControl[PID_PITCH]);

	flightControl[PID_ROLL].kP = proportionalGain[PID_ROLL];
	flightControl[PID_ROLL].kI = integralGain[PID_ROLL];
	flightControl[PID_ROLL].kD = derivativeGain[PID_ROLL];
	flightControl[PID_ROLL].setPoint = 0.0f;
	flightControl[PID_ROLL].deltaTime = (1.0f / (float)CONTROL_RATE);
	PID_Initialize(&flightControl[PID_ROLL]);

	flightControl[PID_YAW].kP = proportionalGain[PID_YAW];
	flightControl[PID_YAW].kI = integralGain[PID_YAW];
	flightControl[PID_YAW].kD = derivativeGain[PID_YAW];
	flightControl[PID_YAW].setPoint = 0.0f;
	flightControl[PID_YAW].deltaTime = (1.0f / (float)CONTROL_RATE);
	PID_Initialize(&flightControl[PID_YAW]);

	flightControl[PID_ALT].kP = proportionalGain[PID_ALT];
	flightControl[PID_ALT].kI = integralGain[PID_ALT];
	flightControl[PID_ALT].kD = derivativeGain[PID_ALT];
	flightControl[PID_ALT].setPoint = 0.0f;
	flightControl[PID_ALT].deltaTime = (1.0f / (float)CONTROL_RATE);
	PID_Initialize(&flightControl[PID_ALT]);

	return HAL_OK;
}

int InitPWMOutput()
{
	/*
	 * Timer used for motor ESC control
	 */
	PWMtimer.numChannels = 4;
	PWMtimer.frequency = PWM_FREQ;
	PWMtimer.TIM = TIM5;
	PWMtimer.Channel = TIM_CHANNEL_ALL;
	PWMtimer.timer = Initialize_PWM(&PWMtimer);

	return HAL_OK;
}

float mapVal(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x > in_max) x = in_max;
	if(x < in_min) x = in_min;

	return ((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}
