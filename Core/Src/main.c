/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "E28.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buf[64]={0};
uint8_t receive_flag=0;//1表示有数�???
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
// {
//     if(GPIO_Pin==AUX_Pin&&issendcmd_flag==0)
//     {
//         receive_flag=1;
//     }
// }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
          uint8_t a=0x80;
		uint8_t b=0x00;
 My_E28_2G4M20S_init();
 My_E28_receive_cmd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
	  
//	  uint8_t tx[2] = {0xC0, 0x00};
//uint8_t rx[2] = {0};

//My_spi_nss_en();
//HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
//My_spi_nss_close();

//volatile uint8_t status = rx[1];   // �⣡�ţ��ǣ�Status��
//volatile uint8_t busy   = status & 0x01;

	  
	   My_E28_receive_cmd();
   // 1. 获取中断状�?? (GetIrqStatus 指令: 0x15)
    uint8_t get_irq_cmd[4] = {0x15, 0x00, 0x00, 0x00}; // 指令 + 1个状态字�?? + 2个空字节换取数据
    uint8_t irq_res[4] = {0};
    
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_TransmitReceive(&hspi1, get_irq_cmd, irq_res, 4, 10);
    My_spi_nss_close();
	HAL_Delay(5);
    // irq_res[2] �?? IRQ 的高 8 位，irq_res[3] �?? IRQ 的低 8 �??
    // RxDone 位在�?? 8 位的�?? 1 位（二进�?? 0000 0010，即 0x02�??
    uint16_t irq_status = (irq_res[2] << 8) | irq_res[3];

		uint8_t c0_cmd[2] = {0xC0, 0x00};
		uint8_t c0_res[2] = {0};
		My_spi_nss_en();
		HAL_Delay(5);
		HAL_SPI_TransmitReceive(&hspi1, c0_cmd, c0_res, 2, 10);
		My_spi_nss_close();
		HAL_Delay(5);
	
    if ((c0_res[1] & 0x01) == 0) {
    // ֻ�в�æ��ʱ�򣬲�ȥ�� IRQ
    //////////////////////////
    uint16_t irq = My_E28_GetIrqStatus_Logic();
	
    if (irq & 0x0002) { 
        // --- 发现 RxDone，说明收到包了！ ---
        
        // 2. 获取接收到的数据长度和起始地�?? (GetRxBufferStatus: 0x17)
        uint8_t get_buf_cmd[4] = {0x17, 0x00, 0x00, 0x00};
        uint8_t buf_info[4] = {0xff,0xff,0xff,0xff};
        My_spi_nss_en();
		HAL_Delay(5);
        HAL_SPI_TransmitReceive(&hspi1, get_buf_cmd, buf_info, 4, 10);
        My_spi_nss_close();
		HAL_Delay(5);
        
        uint8_t payloadLen = buf_info[2]; // 收到多少字节
        uint8_t startAddr  = buf_info[3]; // 数据存在芯片内存的哪�??
        
        // 3. 从芯片内存读取真实数�?? (ReadBuffer: 0x1B)
		uint8_t tx[100] = {0};
		uint8_t rx[100] = {0};

		tx[0] = 0x1B;
		tx[1] = startAddr;
		tx[2] = 0x00; // dummy

		My_spi_nss_en();
		HAL_Delay(5);
		HAL_SPI_TransmitReceive(&hspi1, tx, rx, payloadLen + 3, 100);
		My_spi_nss_close();
		HAL_Delay(5);
        // 此时 real_payload[3] �??后就是你收到的数据了�??
		
		//这里把receivebuffer清空为0x60
		uint8_t test60[30];
		uint8_t tset2[30];
		uint8_t test3[30];
		for(int i=0;i<30;i++)test60[i]=0x60;
		My_E28_writebuffer(test60,30);

		My_readbuffer(a);
		My_readbuffer(b);
        // 4. 重要！清除中断标�?? (ClearIrqStatus: 0x07)
        // 如果不清除，irq_status & 0x02 永远为真，程序会死循环在这里
        uint8_t clear_irq[] = {0x97, 0xFF, 0xFF}; 
        My_spi_nss_en();
		HAL_Delay(5);
        HAL_SPI_Transmit(&hspi1, clear_irq, 3, 10);
        My_spi_nss_close();
		HAL_Delay(5);
        
        // 5. 如果不是持续接收模式，可能需要重新发�?? SetRx 
    }
	
}
    // 稍微歇一下，不要�?? SPI 总线跑太热，也可以不�??
    HAL_Delay(20); 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
