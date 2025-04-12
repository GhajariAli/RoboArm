#include "stepper.h"

uint32_t StepCount=0;
uint8_t StepperState=1;

uint8_t MoveStepperL296(Direction Dir,uint32_t Steps){

	if(Dir==Stop) return 0;

	else if (Dir==Forward){
		if (StepperState<8 && StepperState>0) StepperState++;
		else if (StepperState==8) StepperState=1;
	}
	else if(Dir == Reverse){
		if (StepperState>1) StepperState--;
		else if (StepperState==1) StepperState=8;
	}

	switch (StepperState){
		  case 1:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ4_GPIO_Port, StepperQ4_Pin, 0);
			  break;
		  case 2:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ2_GPIO_Port, StepperQ2_Pin, 1);
			  break;
		  case 3:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ1_GPIO_Port, StepperQ1_Pin, 0);
			  break;
		  case 4:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ3_GPIO_Port, StepperQ3_Pin, 1);
			  break;
		  case 5:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ2_GPIO_Port, StepperQ2_Pin, 0);
			  break;
		  case 6:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ4_GPIO_Port, StepperQ4_Pin, 1);
			  break;
		  case 7:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ3_GPIO_Port, StepperQ3_Pin, 0);
			  break;
		  case 8:
			  StepCount++;
			  HAL_GPIO_WritePin(StepperQ1_GPIO_Port, StepperQ1_Pin, 1);
			  break;
		  default:
			  StepperState=1;
			  HAL_GPIO_WritePin(StepperQ1_GPIO_Port, StepperQ1_Pin, 0);
			  HAL_GPIO_WritePin(StepperQ2_GPIO_Port, StepperQ2_Pin, 0);
			  HAL_GPIO_WritePin(StepperQ3_GPIO_Port, StepperQ3_Pin, 0);
			  HAL_GPIO_WritePin(StepperQ4_GPIO_Port, StepperQ4_Pin, 0);
			  break;
    }
	if(StepCount==Steps) {
		StepCount=0;
		HAL_GPIO_WritePin(StepperQ1_GPIO_Port, StepperQ1_Pin, 0);
		HAL_GPIO_WritePin(StepperQ2_GPIO_Port, StepperQ2_Pin, 0);
		HAL_GPIO_WritePin(StepperQ3_GPIO_Port, StepperQ3_Pin, 0);
		HAL_GPIO_WritePin(StepperQ4_GPIO_Port, StepperQ4_Pin, 0);
		return 1;
	}
	else return 0;
}


uint8_t MoveStepperTMC2209(Direction Dir,uint32_t Steps){
	if(Dir==Stop) return 0;
	else if (Dir==Forward){
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
	}
	else if(Dir == Reverse){
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
	}

	HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
	StepCount++;
	if(StepCount==Steps) {
		StepCount=0;
		return 1;
	}
	return 0;
}
