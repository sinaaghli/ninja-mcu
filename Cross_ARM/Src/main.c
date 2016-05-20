/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  DESCRIPTION : 
  This program achieves three important functionalities :
  1) Data acquisition from Sensors through ADC and IMU.
  2) Driving of a 3 phase BLDC motor and a servo motor.
  3) Taking commands through UART and sending sensor data through UART
  The HAL driver from STM32CUBEMX was used to write the main.c
  There are no interrupts enabled and all DMA interrupts are to be disabled
  in the dma.c  
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* average PWM frequency assumed to be 25 khz. For more information
check the the timer frequency and the prescaler of the motor pwm pins */
#define pwm_period 400

/* conditional compiling for including sensor encoding */
#define ENCODE

HAL_StatusTypeDef status2;                                      // return type of functions, mainly used for debugging

#define packet_size 13*sizeof(uint32_t)                         // size of the packet to be sent through UART(sensor data)
#define headers 5                                               // additional overhead of the packet

#define pulse_1 TIM1->CCR1       //n2                           // compare registers of channels for motor pwm
#define pulse_2 TIM1->CCR2       //n3
#define pulse_3 TIM1->CCR4       //n1

/* initialize led gpio pin */
#define Led_gpio      GPIOE
#define Led_pin       GPIO_PIN_2
 
/* encoder pins*/
#define sens_gpio     GPIOD
#define sens_ph1_pin  GPIO_PIN_11
#define sens_ph2_pin  GPIO_PIN_10
#define sens_ph3_pin  GPIO_PIN_9

/* Motor controller dedicated pins */
#define DcCal_gpio    GPIOD
#define DcCal_pin     GPIO_PIN_7
#define EnGate_gpio   GPIOB
#define EnGate_pin    GPIO_PIN_8

/* ADC PIN and Port Initialization */

#define pot_port      GPIOC
#define pot_1         GPIO_PIN_0      // 1
#define pot_2         GPIO_PIN_1
#define pot_3         GPIO_PIN_2
#define pot_4         GPIO_PIN_3
#define pot_5         GPIO_PIN_4
#define pot_6         GPIO_PIN_5        // 6

#define bat_term      GPIOA             // battery voltage
#define bat_term_pin  GPIO_PIN_4        // 0

/* pins for motor pwm */

#define Phase_gpioH    GPIOB
#define ph_u_h        GPIO_PIN_0

#define ph_v_h        GPIO_PIN_7

#define ph_w_h        GPIO_PIN_1

#define Phase_gpioL    GPIOE
#define ph_u_l        GPIO_PIN_14         //N1
#define ph_v_l        GPIO_PIN_11         //N3
#define ph_w_l        GPIO_PIN_9          //N2

unsigned char packet[packet_size + headers];        // total size of packet

unsigned char state = 0;                  // rotor state of bldc
int cnt = 0;
uint8_t status_i2c = 0;                 

/* volatile buffers for data collection from sensors using or not using DMA */
 __IO uint16_t imu_data[6];

 __IO int8_t recev;                 

 __IO uint8_t accel_data[6];
 __IO uint8_t gyro_data[6];
 
 __IO uint32_t adc_values[7];

 __IO uint32_t sens51[8];
 __IO uint32_t sens52[8];
 __IO uint32_t sens31[8];
 __IO uint32_t sens32[8];
 __IO uint32_t sens41[8];
 __IO uint32_t sens42[8];
 __IO uint32_t sens21[8];
 __IO uint32_t sens22[8];

/* initialize adc values with FF */
void adc_init_values()
{
  uint8_t i = 0;
  for(i=0;i<7;i++)
  {
    adc_values[i] = 0xFFFFFFFF;
  }
}

/* This function returns NULL and is called inside the while 1 loop and is responsible for 
reading IMU data and putting it in the IMU data buffer */
void imu_data_collect()
{    
    HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3B,1,accel_data,6,1);   
    HAL_I2C_Mem_Read(&hi2c3,0xD1,0x43,1,gyro_data,6,1);
    imu_data[0] = ((accel_data[0] << 8) | accel_data[1]);       // 16 bit IMU data read as 8 bit and saved as 16 bit
    imu_data[1] = ((accel_data[2] << 8) | accel_data[3]);
    imu_data[2] = ((accel_data[4] << 8) | accel_data[5]);
    imu_data[3] = ((gyro_data[0] << 8) | gyro_data[1]);
    imu_data[4] = ((gyro_data[2] << 8) | gyro_data[3]);
    imu_data[5] = ((gyro_data[4] << 8) | gyro_data[5]); 
}

/* This function returns NULL and is called inside the while 1 loop and is responsible for 
packet formation using all the sensor data and also adding a checksum to it. The packet also has
two start bytes and two stop bytes The format for the packet is
  "A A ADC0 ADC1 ADC2 ADC3 ADC4 ADC5 ADC6 IMU0 IMU1 IMU2 .... IMU5 checksum 5 5 " */
void packet_form(uint32_t *adc_d, uint32_t *imu_d)
{
  uint8_t count_packet = 0;
  uint8_t packet_current = 2;
  uint32_t sum_protect = 0;
  uint8_t count_data = 0;
  packet[0] = 'A';
  packet[1] = 'A';
  for(count_packet = 0; count_packet < (7*sizeof(uint32_t)); count_packet+=4)
  {
    /* 32 bit data to be saved in 4 8 bit locations */
    packet[packet_current+count_packet] = (unsigned char)(*(adc_d+count_data)>>24);
    packet[packet_current+count_packet+1] = (unsigned char)(*(adc_d+count_data)>>16);
    packet[packet_current+count_packet+2] = (unsigned char)(*(adc_d+count_data)>>8);
    packet[packet_current+count_packet+3] = (unsigned char)(*(adc_d+count_data));
    count_data++;
  }
  count_data = 0;
  packet_current += count_packet;
  for(count_packet = 0; count_packet < 6*sizeof(uint32_t); count_packet+=2)
  {
    /* 16 bit data to be saved in 2 8 bit locations */
    packet[packet_current+count_packet] = (unsigned char)(*(imu_d+count_data)>>8);
    packet[packet_current+count_packet+3] = (unsigned char)*(imu_d+count_data);
    count_data++;
  }
  packet_current += count_packet;
  packet[packet_current+1] = '5';
  packet[packet_current+2] = '5';
  /* checksum is sum of all data */
  for(count_packet = 2; count_packet < (packet_size + headers - 3); count_packet++)
  {
    sum_protect += packet[count_packet];
  }
  packet[packet_current] = (unsigned char)sum_protect;
}

/* This function returns NULL and is used to calculate the value of the duty cycle 
and also assign the duty cycle to a particular pwm pin dedicated for the motor */
void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty)
{
  /* calculation of duty cycle relative to period */
  uint8_t duty_cycle_motor = 0;
  duty_cycle_motor = (duty * pwm_period)/127; 
  /* if pin state is set apply the duty cycle value to the compare register of the particular pin */
  if(PinState == GPIO_PIN_SET)
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
          pulse_3 = duty_cycle_motor;
        break;

        case(ph_v_l):
          pulse_2 = duty_cycle_motor;
        break;

        case(ph_w_l):
          pulse_1 = duty_cycle_motor;
        break;

        default:
        break;
    }
  } 
  /* if pin state is reset then reset the pin */
  else 
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
         pulse_3 = 0;
        break;

        case(ph_v_l):
          pulse_2 = 0;
        break;

        case(ph_w_l):
          pulse_1 = 0;
        break;

        default:
        break;
      }
  }
}
/* This function returns NULL and is mostly responsible for driving the BLDC motor in forward direction
   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.*/
void motor_rotate_forward(char sens, uint8_t duty_motor)
{
    switch(sens) {
      case 5:   
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_SET, duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_SET);
        break;
      case 4:   
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 6:   
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 2:   
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_SET);
        break;
      case 3:   
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_SET);
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        break;
      case 1: 
        HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_SET);
        break;
      default:
        while(1);
      }
}

/* This function returns NULL and is mostly responsible for driving the BLDC motor in reverse direction
   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.*/
void motor_rotate_backward(char sens, uint8_t duty_motor) {
  switch(sens) {
    case 4:   
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_SET);
      
      break;
    case 6:   
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_SET);
      GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);      
      break;
    case 2:   
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_SET);      
      break;
    case 3:   
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_SET);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      
      break;
    case 1:   
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_SET);
      break;
    case 5: 
      HAL_GPIO_WritePin(Phase_gpioH,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(Phase_gpioH,ph_w_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Phase_gpioH,ph_v_h,GPIO_PIN_SET);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      break;
    default:
      while(1);
    }
}

void SystemClock_Config(void);
/* main loop */
int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* i2c based local variables */
  uint8_t i2c_id = 0;
  uint8_t send_i2c = 0;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();         // all BLDC motor pwm
  /* Timers to count encoder ticks */
//  MX_TIM2_Init();       
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  MX_TIM9_Init();       // Servo motor control pwm
  MX_USART3_UART_Init();
  
  adc_init_values();
 /* Start timer 1 pwm for all mosfets */
  HAL_TIM_Base_Start(&htim1);

  /* PWM setup for Nchannel MOSFETS */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
                           
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);                         // Timer to setup PWM for front wheel servo motor

  /* Compile if Encoder Connected */

  #ifndef ENCODE
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Encoder_Start_DMA(&htim5,TIM_CHANNEL_ALL,sens51,sens52,8);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Encoder_Start_DMA(&htim3,TIM_CHANNEL_ALL,sens31,sens32,8);
 
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Encoder_Start_DMA(&htim4,TIM_CHANNEL_ALL,sens41,sens42,8);
 
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Encoder_Start_DMA(&htim2,TIM_CHANNEL_ALL,sens21,sens22,8);
 #endif
 
  /* I2C initialization */
 
 /* read the device ID*/
  status2=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x75,1,&i2c_id,1,10);
  HAL_Delay(10);
  /* Accelerometer and Gyroscope Initialization for resolution and sensitivity, see datasheet */
  send_i2c=0x00;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x6B,1,&send_i2c,1,10);
  send_i2c=0x10;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x1C,1,&send_i2c,1,10);
  send_i2c=0x18;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x18,1,&send_i2c,1,10); 

  /* Initialize enable pins of motor controller */
  HAL_GPIO_WritePin(DcCal_gpio,DcCal_pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EnGate_gpio,EnGate_pin,GPIO_PIN_SET);

  /* initial duty cycle value received through UART 
    format : -128 to 127 
    -128 to -1 : reverse direction of motor 
    0 : stop
    1 to 127 : forward direction of motor */
  recev = 20;
  /* Transmit the packet continuously through DMA */
  HAL_UART_Transmit_DMA(&huart3,(uint8_t *)packet, sizeof(packet));
  /* Receive the command, one byte of signed integer value */
  HAL_UART_Receive_DMA(&huart3,(int8_t *)&recev,sizeof(int8_t));
  /* Start ADC conversions and save the value to memory through DMA. 7 channels 7 memory positions*/
  HAL_ADC_Start_DMA(&hadc1,adc_values,7);
  /* Switch on the LED for one second */
  HAL_GPIO_WritePin(Led_gpio,Led_pin,GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(Led_gpio,Led_pin,GPIO_PIN_RESET);

  while (1)
  { 
    /* get the rotor state of the bldc */
    state = HAL_GPIO_ReadPin(sens_gpio,sens_ph1_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph2_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph3_pin);
    
    /* IMU data collection and packet formation */        
    imu_data_collect();
    packet_form(adc_values,&imu_data); 
    
    /* There is no interrupt involved with UART hence its mostly polling
       if recev is positive then direction is forward, incase of negative,
        direction is reverse */
    if(recev >= 0)
    {
      motor_rotate_forward(state, abs(recev));   
    }
    else
    {
      motor_rotate_backward(state, abs(recev));
    }
  }
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
