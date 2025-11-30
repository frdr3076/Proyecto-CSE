/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOCAL_ADDR 0x05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t vec[]="pruebademensaje";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void receive_sensor(uint8_t *);
uint8_t send_data(uint8_t *, uint8_t ,uint8_t ,uint8_t );
uint8_t receive_data(void);
int compareTwoString(char a[],char b[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_MspInit(&huart1);//Inicializar la comunicación
  HAL_UART_MspInit(&huart2);

  send_data(vec,LOCAL_ADDR,0x06,sizeof(vec));
  receive_data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  receive_data();


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(TR_EN_GPIO_Port, TR_EN_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : TR_EN_Pin */
  GPIO_InitStruct.Pin = TR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TR_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void receive_sensor(uint8_t *data){
	uint8_t payload[16]={0};
	uint32_t i,j;

	for(i=0;i<=sizeof(*data);i++){
		if(*(data+i)=='A' && *(data+i+1)=='A'){
			for(j=0;j<16;j++){
				if(*(data+j)!='B'&& *(data+j+1)!='B')
					*(payload + j) = *(data + i + j + 2);
				else {
					*(payload+j-1)=0;
					break;
				}
			}
		}

	}

return;
}
uint8_t send_data(uint8_t *payload, uint8_t orAddr,uint8_t destAddr,uint8_t size){
	uint8_t START_FRAME[2]= {0xAA,0x55};
	uint8_t END_FRAME[2]= {0x55,0xAA};
	uint8_t maxSize=11+size;
	uint8_t crc[4]={0};
	uint32_t aux_crc=0;
	uint8_t Cant_Ceros_add=0;

	Cant_Ceros_add=4-(11+size)%4;	//determinamos la canitdad de cero a agregar para hacerlo multiplo de 4

	uint8_t trama[maxSize+Cant_Ceros_add];
	unsigned int i;
	unsigned int j;
	for(i=0;i< maxSize;i++){
		if(i==0)
			*(trama+i)= START_FRAME[i];
		if(i==1)
			*(trama+i)= START_FRAME[i];
		if(i==2)
			*(trama+i)= orAddr;
		if(i==3)
			*(trama+i)= destAddr;
		if(i==4)
			*(trama+i)= size;
		if(i==5){
			for(j=0;j<size;j++)
				*(trama+i+j)=*(payload+j);
		}
		if(i==6){
			for(j=0;j<4;j++){
				*(trama+i+j+size-1)=*(crc+j);		//completamos con '0' las posiciones del CRC
			}
		}
		if(i==7)
			*(trama+i+size+2)= END_FRAME[0];
		if(i==8){
				*(trama+i+size+2)= END_FRAME[1]; //
			for(j=0;j<Cant_Ceros_add;j++)
				*(trama+i+j+1+size+2)=0x00;		//agregamos los ceros adicionales al final de la trama para que sea multiplo de 32bits
		}
	}
	aux_crc= HAL_CRC_Calculate(&hcrc,(uint32_t *) trama, sizeof(trama)/sizeof(uint32_t));//calculamos el CRC

	*(trama+8+size)=aux_crc&0xFF000000>>24;			//dividimos los 32 bits del CRC...
	*(trama+7+size)=(aux_crc&0xFF000000>>16)>>8;	//...y lo colocamos en el vector Trama
	*(trama+6+size)=(aux_crc&0xFF000000>>8)>>16;
	*(trama+5+size)=(aux_crc&0xFF000000)>>24;

	 HAL_GPIO_WritePin(TR_EN_GPIO_Port, TR_EN_Pin,GPIO_PIN_SET );	//transmision enable
	 HAL_Delay(1);
	 HAL_UART_Transmit(&huart1, trama, sizeof(trama), HAL_MAX_DELAY);
	 HAL_GPIO_WritePin(TR_EN_GPIO_Port, TR_EN_Pin,GPIO_PIN_RESET );	//reception enable
	return 0;

}
uint8_t receive_data(void){
	uint8_t buffer=0;
	uint8_t state=0;
	uint8_t orAddr;
	static uint8_t i=0;
	static uint8_t payload_size=0;
	static uint8_t end_data =0;
	uint8_t payload[256]={0};
	uint8_t trama[28]={0};
	unsigned int status=HAL_OK;
	int j=0;
	uint8_t aux_crc[4]={0};
	uint32_t crc_rx_calc=0;
	uint32_t crc_rx_recib=0;
	while(!end_data){
		status=HAL_UART_Receive(&huart1, &buffer, sizeof(buffer), 2000);
		if(status==HAL_TIMEOUT){
			send_data(vec,LOCAL_ADDR,0x06,sizeof(vec));
			return 0;
		}
		j++;
			switch(state){
				case 0:
					if(buffer==0xAA){
							state++;
							trama[0]=buffer;
					}
				break;
				case 1:
					if(buffer==0x55){
							state++;
							trama[1]=buffer;
					}
						else state=0;
				break;
				case 2:
					orAddr= buffer;
						state++;
						trama[2]=buffer;
				break;
				case 3:
					if(buffer==LOCAL_ADDR){
							state++;
							trama[3]=buffer;
					}
					else state = 0;
				break;
				case 4:
					payload_size=buffer;
					trama[4]=buffer;
						state++;
				break;
				case 5:
					*(payload+i)=buffer;
					trama[5+i]=buffer;
						i++;
						if(i>payload_size-1){
							state ++;
							i=0;
						}
				break;
				case 6:
					trama[5+payload_size+i]=0;
					aux_crc[i]=buffer;
						i++;
						if(i>3){
							state++;
							i=0;
						}
				break;
				case 7:
					if(buffer==0x55){
						state++;
						trama[9+payload_size]=buffer;
					}
						else state=0;
				break;
				case 8:
					if(buffer==0xAA){
							end_data=1;
							trama[10+payload_size]=buffer;
					}
						state=0;
				break;
				default:
					end_data=1;
						 state=0;
				break;
			}

	}

	crc_rx_calc= HAL_CRC_Calculate(&hcrc,(uint32_t *) trama, sizeof(trama)/sizeof(uint32_t));
	crc_rx_recib= aux_crc[3] | aux_crc[2]<<8 | aux_crc[1]<<16 |aux_crc[0]<<24;


	if(trama[5]=='E' && trama[6]=='R')
		send_data(vec,LOCAL_ADDR,0x06,sizeof(vec));
	return 0;
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
