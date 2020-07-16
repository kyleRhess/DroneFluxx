#include "PID.h"


void PID_Initialize(PID_Controller * _PID)
{
	// Init all values to zero
	_PID->sampleError 		= 0.0f;
	_PID->deltaInput 		= 0.0f;
	_PID->lastInput 		= 0.0f;
	_PID->PTerm 			= 0.0f;
	_PID->ITerm 			= 0.0f;
	_PID->DTerm 			= 0.0f;
	_PID->controllerOutput 	= 0.0f;
	_PID->updates 			= 0;
	_PID->windupGuard 		= 100.0f;
}

void PID_Update(PID_Controller * _PID, float systemFeedback)
{
	// Compute error between set-point and system measurement
	_PID->sampleError = _PID->setPoint - systemFeedback;

	// Compute diff from last error
	_PID->deltaInput = systemFeedback - _PID->lastInput;

	// Remember last error value
	_PID->lastInput = systemFeedback;

	// Calculate terms
	_PID->PTerm =  _PID->kP * _PID->sampleError;
	_PID->ITerm += _PID->kI * _PID->deltaTime * _PID->sampleError;
	_PID->DTerm = -_PID->kD * _PID->deltaInput / _PID->deltaTime;

	// Prevent integrator wind-up
	if(_PID->ITerm > _PID->windupGuard)
		_PID->ITerm = _PID->windupGuard;

	if(_PID->ITerm < -_PID->windupGuard)
		_PID->ITerm = -_PID->windupGuard;

	// Sum terms into controller output
	_PID->controllerOutput = _PID->PTerm + _PID->ITerm + _PID->DTerm;

#if 0
	// Clamp output to -100-100% (needs to go negative to subtract from "motorPower" value)
	if(_PID->controllerOutput >  99.99f) _PID->controllerOutput =  99.99f;
	if(_PID->controllerOutput < -99.99f) _PID->controllerOutput = -99.99f;
#endif

	_PID->updates++;
}

void PID_SetSetpoint(PID_Controller * _PID, float newSetpoint)
{
	_PID->setPoint = newSetpoint;
}

float PID_GetOutput(PID_Controller * _PID)
{
	return _PID->controllerOutput;
}

void PID_SetOutput(PID_Controller * _PID, float newOutput)
{
	_PID->controllerOutput = newOutput;
}

void PID_SetKp(PID_Controller * _PID, float proportional_gain)
{
	_PID->kP = proportional_gain;
}

void PID_SetKi(PID_Controller * _PID, float integral_gain)
{
	_PID->kI = integral_gain;
}

void PID_SetKd(PID_Controller * _PID, float derivative_gain)
{
	_PID->kD = derivative_gain;
}

void PID_Reset(PID_Controller * _PID)
{
	_PID->sampleError 		= 0.0f;
	_PID->deltaInput 		= 0.0f;
	_PID->lastInput 		= 0.0f;
	_PID->controllerOutput 	= 0.0f;
	_PID->updates 			= 0;
	_PID->PTerm 			= 0.0f;
	_PID->ITerm 			= 0.0f;
	_PID->DTerm 			= 0.0f;
}
