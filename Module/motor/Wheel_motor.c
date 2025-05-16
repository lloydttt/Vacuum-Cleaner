// /******************************************************************************
//  * @file    Wheel_motor.c
//  * @brief   Motor of vaccum cleaner dirving source code.
//  * @author  YanTao Lloyd Lai (lloydt@qq.com)
//  * @version 1.0
//  * @date    2025-4-14
//  *
//  * @copyright
//  * Copyright (c) 2025 Vacuum Cleaner Project. All rights reserved.
//  ******************************************************************************/



// /******************************************************************************
//  * @brief  PWM output control function
//  * @param  None 
//  * @note   None
//  ******************************************************************************/
// void PWM_Control(uint16_t speed){



// } 





























// //     // static int flag = 0;
// //     // Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);  
// //     // enc1 = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));	//获取定时器的值

// // 	  // if((Direction == 0) &(enc1 < enc1_old))				//正向旋转数值变小,说明进位
// // 	  // {
// // 	  // 	enc2++;
// // 	  // }
// // 	  // if((Direction == 1) &(enc1 > enc1_old))				//反向旋转数值变小,说明借位
// // 	  // {
// // 	  // 	enc2--;
// // 	  // }
// // 	  // enc1_old = enc1;									//更新enc1_old，便于下次计算
// // 	  // enc = enc2 << 16 | enc1;								//计算当前计数总值，带+-号
// // 	  // counter++;											//主函数计数
// // 	  // if(counter>1000)									//主函数每次运行约1ms，此处用于每1000ms发送一次数
// // 	  // {
// // 	  // 	counter = 0;									//计数值清零
// // 	  // 	printf("Dir %d,	Enc2 %d, Enc1 %d, ENC %d\r\n",Direction,enc2,enc1,enc);//打印相关计数数据
// //     //   flag = !flag;
// // 	  // }
// // 	  // HAL_Delay(1);
// //     // if(flag == 1)
// //     //   HAL_GPIO_WritePin(MOTOR1_DRC_GPIO_Port, MOTOR1_DRC_Pin, GPIO_PIN_SET);
// //     // else if(flag == 0)
// //     //   HAL_GPIO_WritePin(MOTOR1_DRC_GPIO_Port, MOTOR1_DRC_Pin, GPIO_PIN_RESET);