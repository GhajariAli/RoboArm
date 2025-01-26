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

uint8_t MoveStepper(Direction Dir,uint32_t Steps);


#endif /* INC_STEPPER_H_ */
