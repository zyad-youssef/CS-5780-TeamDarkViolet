
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Public variables ---------------------------------------------------------*/
uint32_t timer2 = 0;
uint32_t timer1 = 0;
//the calculated distance
int timeElapsed = 0; 
int obstacle_found=0;
void SystemClock_Config(void);
void go_forward(void);
void stop(void);
void SystemClock_Config(void);





int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	//enable gpio clocks
	RCC->AHBENR |= (1<<19) | (1<<17);
	//en TIM2 clock
  RCC->APB1ENR |= (1<<0); 
	
	//initialize all leds
	GPIOC->MODER |= (1<<16) | (1<<18)| (1<<14) | (1<<12);

 //set pc0 to be output, pull up, 
    GPIOC->MODER |= GPIO_MODER_MODER0_0;
    GPIOC->MODER &= ~(GPIO_MODER_MODER0_1);
    //set to pull up
    GPIOC->PUPDR |= (1<<0);
    GPIOC->PUPDR &= ~(0);
      
    // alternate function mode for PA1 for the echo pin
		GPIOA->MODER |= (1<<3);

		//set PA1 to pull up
    GPIOA->PUPDR |= (1<<2) | (1<<3);

    
    GPIOA -> AFR[0] |= (1<<5);
    
    TIM2 ->PSC = 9;
    TIM2 ->ARR = 65000;
    
    
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
    
    TIM2->CCER |= TIM_CCER_CC2P;
    TIM2->CCER |= TIM_CCER_CC2NP;
    
    TIM2->CCER |= TIM_CCER_CC2E;
    TIM2->DIER |= TIM_DIER_CC2IE;
		
    TIM2->CR1 |= (1<<0);

		
		//configure the motor

		GPIOC->MODER |= GPIO_MODER_MODER3_0;
		
    GPIOC->MODER &= ~(GPIO_MODER_MODER3_1);
    GPIOC->MODER |= GPIO_MODER_MODER4_0;
		
    GPIOC->MODER &= ~(GPIO_MODER_MODER4_1);
		
				//change the priority of tim2
    NVIC_EnableIRQ(TIM2_IRQn); 
		NVIC_SetPriority(TIM2_IRQn,0);
		
		
		

		
  while (1)
  {
		 GPIOC -> ODR |= GPIO_ODR_0;
	
    //start delay 
		static uint32_t i = 0, j = 0;
    for(j=0; j<15; j++)
        for(i=0; i<1; i++);
		//end delay 
	
	
    GPIOC -> ODR &= ~(GPIO_ODR_0);
    HAL_Delay(60);
		
		if(obstacle_found) {
			stop();
		}
		else
			go_forward();
	
		
  } /* while-loop ends */
} /* main ends */



/* Hardware interrupt for TIM2 */
void TIM2_IRQHandler(void){
    int timeDiff = 2500;
    if (timeElapsed == 0) {
            
            timer1 = TIM2 -> CCR2;
            timeElapsed = 1;
   
        }    
  else if (timeElapsed == 1)
        {

        timer2 = TIM2 -> CCR2;
        uint32_t temp = 0;
        if(timer2 < timer1)
        {
					temp = (timeDiff - timer1) + timer2;
        }
        else if(timer1 < timer2)
        {
					temp = timer2 - timer1;
        }
				
        if(temp == 0)
        {
          GPIOC -> ODR = (1<<9);
        }
				
        uint32_t distance = temp * .034/2;
        timeElapsed = 0;
        if(distance <= 10)
        {
            obstacle_found = 1; 
        }
        else if(distance > 10){
           obstacle_found = 0;
        }
    }    
    TIM2 -> SR =0;
}


/* Turn off both of the motors */
void stop(){
		
		GPIOC->ODR &= ~(GPIO_ODR_9);
		GPIOC->ODR &= ~(GPIO_ODR_3);
			//turn red led on when obstacle is close
		GPIOC->ODR |= GPIO_ODR_6;
		//GPIOC->ODR &= ~(GPIO_ODR_9);
		
		GPIOC->ODR &= ~(GPIO_ODR_4);
}
	
void go_forward(){
		
		GPIOC->ODR |= GPIO_ODR_9;
		GPIOC->ODR |= GPIO_ODR_3;
	//turn red led off
		GPIOC->ODR &= ~(GPIO_ODR_6);
		//GPIOC->ODR |= GPIO_ODR_9;
		
		GPIOC->ODR |= GPIO_ODR_4;
		
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/