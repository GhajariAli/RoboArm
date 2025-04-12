#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "stdint.h"
#include "main.h"

typedef enum {
	Reverse,
	Forward,
	Stop
}Direction;


extern uint32_t StepCount;
extern uint8_t StepperState;

uint8_t MoveStepperL296(Direction Dir,uint32_t Steps);

#define StepperQ1_GPIO_Port 0
#define StepperQ2_GPIO_Port 0
#define StepperQ3_GPIO_Port 0
#define StepperQ4_GPIO_Port 0
#define StepperQ1_Pin 0
#define StepperQ2_Pin 0
#define StepperQ3_Pin 0
#define StepperQ4_Pin 0
#endif /* INC_STEPPER_H_ */
