#include "PID.h"


void PID_Initialize(PID_Controller * _PID)
{
	// Init all values to zero
	_PID->sampleError 		= 0.0f;
	_PID->deltaInput 		= 0.0f;
	_PID->lastError 		= 0.0f;
	_PID->PTerm 			= 0.0f;
	_PID->ITerm 			= 0.0f;
	_PID->DTerm 			= 0.0f;
	_PID->controllerOutput 	= 0.0f;
	_PID->updates 			= 0;
	_PID->windupMax			= 100.0f;
	_PID->windupMin			= -100.0f;
}

void PID_Update(PID_Controller * _PID, float systemFeedback)
{
	float new_i = 0.0f;
	int windGaurd = 1;

	// Compute error between set-point and system measurement
	_PID->sampleError = _PID->setPoint - systemFeedback;

	// Compute diff from last error
	if(_PID->kD > 0.0f)
		_PID->deltaInput = _PID->lastError - _PID->sampleError;

	// Remember last error value
	_PID->lastError = _PID->sampleError;

	// Calculate terms
	_PID->PTerm =  _PID->kP * _PID->sampleError;
	new_i =  _PID->ITerm + (_PID->kI * _PID->deltaTime * _PID->sampleError);

	if(_PID->kD > 0.0f)
		_PID->DTerm = _PID->kD * (_PID->deltaInput / _PID->deltaTime);

	// Sum terms into controller output
	_PID->controllerOutput = _PID->PTerm + new_i + _PID->DTerm;

	// Prevent integrator wind-up
	if(_PID->controllerOutput > _PID->windupMax)
	{
		_PID->controllerOutput = _PID->windupMax;

		if(_PID->sampleError > 0.0f)
		{
			windGaurd = 0;
		}
	}
	else if(_PID->controllerOutput < _PID->windupMin)
	{
		_PID->controllerOutput = _PID->windupMin;

		if(_PID->sampleError < 0.0f)
		{
			windGaurd = 0;
		}
	}

	if(windGaurd == 1)
		_PID->ITerm = new_i;

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
	_PID->lastError 		= 0.0f;
	_PID->controllerOutput 	= 0.0f;
	_PID->updates 			= 0;
	_PID->PTerm 			= 0.0f;
	_PID->ITerm 			= 0.0f;
	_PID->DTerm 			= 0.0f;
}
