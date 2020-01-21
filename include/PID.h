#ifndef PID_H_ /* include guard */
#define PID_H_

#include <stdio.h>

typedef struct PID_Control
{
	float kP;
	float kI;
	float kD;
	float deltaTime;
	float setPoint;

	uint32_t updates; /*I know you wont listen, but don't ever touch these members.*/
	float sampleError;/*I know you wont listen, but don't ever touch these members.*/
	float deltaInput;/*I know you wont listen, but don't ever touch these members.*/
	float lastInput;/*I know you wont listen, but don't ever touch these members.*/
	float PTerm;/*I know you wont listen, but don't ever touch these members.*/
	float ITerm;/*I know you wont listen, but don't ever touch these members.*/
	float DTerm;/*I know you wont listen, but don't ever touch these members.*/
	float controllerOutput;/*I know you wont listen, but don't ever touch these members.*/
	float windupGuard;/*I know you wont listen, but don't ever touch these members.*/
} PID_Controller;

void PID_Initialize(PID_Controller * _PID);
void PID_Update(PID_Controller * _PID, float systemFeedback);
void PID_SetSetpoint(PID_Controller * _PID, float newSetpoint);
void PID_SetKp(PID_Controller * _PID, float proportional_gain);
void PID_SetKi(PID_Controller * _PID, float integral_gain);
void PID_SetKd(PID_Controller * _PID, float derivative_gain);
void PID_Reset(PID_Controller * _PID);
float PID_GetOutput(PID_Controller * _PID);
void PID_SetOutput(PID_Controller * _PID, float newOutput);

#endif /* PID_H_ */
