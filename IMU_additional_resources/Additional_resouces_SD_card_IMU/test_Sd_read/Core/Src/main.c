/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned int Timer_flag= 0;
unsigned int i_sd_write = 1;
unsigned int sd_fram_size=100;
unsigned int sd_write_complete=0;
unsigned int sd_frame_number = 10;
unsigned int i_sd_frame = 0;
unsigned int sd_write_counter =1;
unsigned int i_sd_read = 0;
unsigned int act_int_num[100];  //actual data number in each frame
int i_int = 0 ;
int converted = 0 ;
UINT read_int_size=9000;
UINT read_char_size=48500;

char buffer_uart[100]; //writing buffer_uart
UINT br, bw;  // File read/write count

UINT read_point=0;
UINT i_int_previous=0;

char rd_buffer[48510];
int16_t rd_buffer_int[8500];
char buffer[45000];
UINT read_point_previous=0;
UINT read_int_total=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
FRESULT open_append(FIL* fp, const char* path);
FRESULT open_append_read(FIL* fp, unsigned int read_point, const char* path);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void send_uart(char*string)
{
	uint8_t len=strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 2000);
}

int bufsize (char*buf)
{
	int i=0;
	while(*buf++ !='\0') i++;
	return i;
}

void bufclear (char *buffer_uart)
{
	for (int i=0; i<sizeof(buffer_uart); i++)
	{
		buffer_uart[i]='\0';
	}
}

FRESULT open_append(FIL* fp, /* [OUT] File object to create */
const char* path /* [IN]  File name to be opened */
) {

	/* Opens an existing file. If not exist, creates a new file. */
	fresult = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS | FA_WRITE);
	if (fresult == 0) {
		/* Seek to end of the file to append data */
		fresult = f_lseek(fp, f_size(fp));
		if (fresult != 0)
			f_close(fp);
	}
	return fresult;
}
FRESULT open_append_read(FIL* fp, /* [OUT] File object to create */
		unsigned int read_point, /*starting point for each read*/
const char* path /* [IN]  File name to be opened */
) {

	/* Opens an existing file for read. */
	fresult = f_open(fp, path, FA_READ);
	if (fresult == 0) {
		/* Seek to end of the file to append data */
		if(read_point<f_size(&fil))
		{
			fresult = f_lseek(fp, read_point);
		}
		else
		{
			fresult=1;
		}
		if (fresult != 0)
			f_close(fp);
	}
	return fresult;
}
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Mount SD card*/
    fresult = f_mount(&fs, "/", 1);
    if (fresult != FR_OK) send_uart ("ERROR!!! in mounting SD CARD...\n\n");
    else send_uart("SD CARD mounted successfully...\n\n");

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf (buffer_uart, "SD card total size:\t%lu\n", total);
	send_uart(buffer_uart);
	bufclear(buffer_uart);


	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	sprintf (buffer_uart, "SD card free space:\t%lu\n", free_space);
	send_uart(buffer_uart);
	bufclear(buffer_uart);

	read_int_total=0;
	Timer_flag=1;
	//try to use fwrite and fread
	/* Open file to write/ create a file if it doesn't exist */
   // fresult = f_open(&fil, "fileTimer.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

//	//Open file to read
//		strcpy(buffer_uart, "This is File 2 and it says Hello from Aimee \n");
//
//		fresult = f_write(&fil,buffer_uart, bufsize(buffer_uart),&bw);
//
//		send_uart ("FILE2.text created and the data is written\n");
//		/* Close file */
//			fresult = f_close(&fil);
//
//			//Open file to read
//			fresult=f_open(&fil, "file1.txt", FA_READ);
//
//			// read data from the file
//				f_read(&fil, buffer_uart, fil.fptr,&br);
//
//				send_uart(buffer_uart);
//
//				f_close(&fil);
//
//				bufclear();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //test sd read

	  if (Timer_flag)
	  {
		   Timer_flag=0;
		 // fresult=f_open(&fil, "data_8k.txt", FA_READ);
		  fresult= open_append_read(&fil,read_point,"data_8k.txt");
		  if(fresult == FR_OK)
		  {
			  int i = 0;
			  f_read(&fil, rd_buffer, read_char_size,&br);


			  for (i = 0; i < br; i++)
			  {
			      if (rd_buffer[i] == '\n')
			    	  rd_buffer[i] = ',';
			  }
              if ((rd_buffer[i-1]!='\r') && (rd_buffer[i-1]!=',' )) //If a integer has not completely been read
              {
            	  int i_pad=0;
            	  f_read(&fil, &(rd_buffer[i]), 10,&br);
              	  for (i_pad=i;i_pad < read_char_size+br; i_pad++)
             	 {
					  if (rd_buffer[i_pad] == '\n')
					  {
						  rd_buffer[i_pad] = ',';
						  for(int i_clear=i_pad+1;i_clear < read_char_size+br;i_clear++)
							{
							  rd_buffer[i_clear] = ',';
							}
						  break;
					  }

             	 }
              	  br=i_pad-1;  //update br after padding
              }

			  char* tok = rd_buffer ;

				  do
				  {
					  if (*tok == ',')
					  {
						  tok = strchr( tok, ',' ) + 1 ;
						  converted=0;
					  }
					  else
					  {
					  converted = sscanf( tok, "%d", &rd_buffer_int[i_int] ) ;
					  i_int++;
					  tok = strchr( tok, ',' ) + 1 ;
					  }

				  } while( tok != NULL  && converted != -1);


		   f_close(&fil);

///This section is only if you would like to use smaller buffer, and read the audio signal from .txt multiple times
//		   	   	   	   if(br>=read_char_size)
//		  			   {
//		  				   i_int--;
//		  				   i_int_previous=i_int; //save the index in the previous frame
//		  				   read_point_previous=read_point; //save the reading point in the previous frame
//		  				   read_point=read_point+br;  //move the reading point
//
//		  				 for (int i=i_int_previous;i<read_int_size; i++)
//						   {
//							   rd_buffer_int[i]= 0;
//						   }
//						   act_int_num[i_sd_read]=i_int_previous;
//						   i_int=0;
//		  			   }
//		  			   else
//		  			   {
//		  				 //the last frame for the entire reading
//						   read_point=read_point+br; //move the reading point
//						   for (int i=i_int;i<read_int_size; i++)
//							{
//								rd_buffer_int[i]= 0;
//							}
//						   act_int_num[i_sd_read]=i_int;
//		  			   }
//		   	   	   read_int_total=read_int_total+act_int_num[i_sd_read];
//		   		  i_sd_read++;
/////////////////////////////////////////////////////////////////////////////////////////////////////

		   bufclear(rd_buffer);

	  }
		 // else
		  //{
			  		//HAL_TIM_Base_Stop_IT(&htim3);
			  send_uart("SD CARD has been read successfully.\n The first 8k data in rd_buffer_int can be used for processing\n");
			  HAL_Delay(100);
		  //}

			  //test sd write
              int buffer_data[5];
		  	  for (int i=0; i<7500; i++){




		  		  buffer_data[0]=abs(rd_buffer_int[i])/1000; // get the thousandth value
		  		  buffer_data[1]=(abs(rd_buffer_int[i])-1000*buffer_data[0])/100; // get the hundredth value
		  		  buffer_data[2]=(abs(rd_buffer_int[i])-1000*buffer_data[0]-100*buffer_data[1])/10; // get the tenth value
		  		  buffer_data[3]=(abs(rd_buffer_int[i])-1000*buffer_data[0]-100*buffer_data[1]-10*buffer_data[2]); // get the oneth value


		  		 if(rd_buffer_int[i]<0)

				 {
		  			buffer[6*i]='-';
				 }
				 else
				 {
					 buffer[6*i]='0';
				 }

		  		buffer[6*i]= buffer[6*i]; // according to ASCII code, decode to the corresponding displayable  number
		  		buffer[6*i+1]= (buffer_data[0])+0x30;
		  		buffer[6*i+2]= buffer_data[1]+0x30;
		  		buffer[6*i+3]= buffer_data[2]+0x30;
		  		buffer[6*i+4]= buffer_data[3]+0x30;
		  		buffer[6*i+5]=0x0a; //change line

		  	  }

		  	  // save the whole buffer to SD card
		  	 fresult= open_append(&fil,"SDcard_save.txt");
		  				 if(fresult == FR_OK)
		  				 {
		  					 fresult = f_write(&fil,buffer, bufsize(buffer),&bw);
		  					 fresult = f_close(&fil);
		  				 }
	 }
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Timer_flag=1;
}

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
