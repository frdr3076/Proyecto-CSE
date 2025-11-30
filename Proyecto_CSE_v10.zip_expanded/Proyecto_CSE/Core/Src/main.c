/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programa principal de Proyecto Final de Cursada de CSE
  ******************************************************************************
  * @attention
  * DESCRIPCION: Se lee por UART una palabra de 16 hasta 256 bytes que debe comenzar con 'AA55' y finalizar con '55AA' para ser
  *              reconocida. Luego se guarda en memoria a través de comunicación I2C y se envía por UART.
  * UART no tiene incremento en memoria DMA Settings
  *
  * #### PROCEDIMIENTO ####
  * 1) Recibimos datos del sensor. Debe estar compuesta por: AA55 <Mensaje> 55AA
  * 2) Sensor tiene direccion 0x05
  * 3) Lora Side tiene direccion 0x06
  * 4) Protocolo de mensaje: 0xAA 0x55 DIR_ORIG(1 byte) DIR_DEST(1 byte) SIZE(1 byte) PAYLOAD(16 a 256 bytes) CRC(1 byte) 0x55 0xAA
  *
  * ####
  * HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0); // Equivale a poner en GND.
  * ####
  * UART 6 para TX y RX de trama.
  * UART 1 comando AT
  * UART 2 para DEBUG
  * PINTOUT RS485
  * PC6 = DI (TX)
  * PC7 = RO (RX)
  * PC8 = Salida colector abierto digital para configurar Rx/Tx
  *
  * #### Pinout I2C: ####
  * Pin1: A0  Pin8: VCC
  * Pin2: A1  Pin7: WP
  * Pin3: A2  Pin6: SCL (PB8)
  * Pin4: GND Pin5: SDA (PB9)
  *
  * ### Troubleshooting ###
  *  Si se queda esperando el debugger, compilá y depurá otro programa antes que este
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

# define MAX_MSG_LENGTH 256

// ############## Defines I2C ##############
#define CONTROL_CODE 0x0A
#define BLOCK1 0x00
HAL_StatusTypeDef Return_I2C; // Variable de estado de protocolo I2C.

#define ESTADO0 0
#define ESTADO1 1
#define ESTADO2 2
#define ESTADO3 3
#define ESTADO4 4
#define ESTADO5 5
#define ESTADO6 6
#define ESTADO7 7
#define ESTADO8 8
#define ESTADO9 9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

//############## Variables para transmitir mensaje.##############

uint8_t msg_length;         // Contador de letras de mi mensaje
uint8_t input_data[] = "pepepepeepAAHOLA Conectividad de sistemas Embebidos55";

uint8_t dir_origen = 0x5;
uint8_t dir_dest = 0x6;

uint8_t payloadprueba[8]={0x58,0x57,0x56,0x55,0x54,0x53,0x52,0x51};



// Variables para I2C
uint16_t Control_Byte;
uint8_t Data_I2C_Rx[6] ={0}; // size(Data_Tx)-2 porque no quiero leer la posicion de memoria, sino los datos
uint8_t Data_I2C_Tx[8] = {0,4,'A','B','C','D','E','F'}; // primeros bytes corresponden a la cantidad de word address, 24c32 tiene 2 word address.
uint8_t cnt = 0;
uint8_t Direccion[2]= {0,4};
HAL_StatusTypeDef Return;

uint8_t data_rx;
uint8_t info[267];			// 256 de payload + 11 de la trama.
uint8_t RX_PENDING = 1;     // Inicializo en 1


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

uint8_t contar_bytes_ciclicos(const uint8_t *arr,uint8_t pos);
uint8_t CSE_send_data(uint8_t* vec, uint8_t origen, uint8_t destino ,uint8_t size);
void Escritura_I2C(uint8_t *vec,uint8_t msg_length);
uint32_t Escritura_CRC(uint8_t origen, uint8_t destino, uint8_t size, uint8_t* payload_rx);
uint8_t Run_Program();
uint8_t CSE_receive_data();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//### Funcion que cuenta los bytes de payload ###
//uint8_t contar_bytes_ciclicos(const uint8_t *arr , uint8_t pos){
//	uint8_t cant_bytes = 0;
//	int j = 0;
//   	for( j = pos+2; j < MAX_MSG_LENGTH ; j++){
//            if ((arr[j] == '5') && (arr[(j+1)] == '5')){
//            	return cant_bytes;
//           	break;
//            }
//			cant_bytes = cant_bytes + 1;
//		}
//	return cant_bytes;
//}

//### Funcion para encontrar inicio de trama. Parámetros: vector donde buscar y tamaño ###
uint8_t Encontrar_START(uint8_t *vec,uint8_t len){
    int it = 0;
    uint8_t pos = 0;
	for(it = 0; it < len; it++ ){
		if( vec[it]== 'A' && vec[it+1]== 'A'){
			return pos;
		}
		pos = it + 1;
	}
	return pos;
}

//### Funcion que arma el protocolo de transmisión ###
uint8_t CSE_send_data(uint8_t* vec, uint8_t origen , uint8_t destino , uint8_t size){
	uint8_t PACKAGE_BYTES = 11; // Cantidad de bytes adicionales al payload a transmitir.
	uint8_t i=0;
	uint8_t crc [4]= {0x00,0x00,0x00,0x00};
	uint8_t data[size + 11];
	uint32_t CRC_AUX=0;
	for(i=0; i < size + PACKAGE_BYTES; i++)
	{
	  if(i==0)
		  data[i] = 0xAA;
	  if(i==1)
		  data[i] = 0x55;
	  if(i==2)
		  data[i] = origen;
	  if(i==3)
		  data[i] = destino;
	  if(i==4)
		  data[i] = size; // 16
	  if(i==(size+5))
		  data[i] = crc[3];
	  if(i==(size+6))
  		  data[i] = crc[2];
	  if(i==(size+7))
  		  data[i] = crc[1];
	  if(i==(size+8))
  		  data[i] = crc[0];
	  if(i ==(size+9))
		  data[i] = 0x55;
	  if(i == (size+10)){
		  data[i] = 0xAA;
	  }
	  if( (i>=5) && (i<=(5+size)))
		  data[i] = vec[i-5];
	}

	 CRC_AUX=Escritura_CRC(data[2], data[3], data[4], vec);
	 data[size+5]=(CRC_AUX&0xFF000000)>>24;
	 data[size+6]=(CRC_AUX&0x00FF0000)>>16;
	 data[size+7]=(CRC_AUX&0x0000FF00)>>8;
	 data[size+8]=CRC_AUX&0x000000FF;


	HAL_UART_Transmit_DMA(&huart2, data, sizeof(data));
	HAL_UART_Transmit_DMA(&huart6, data, sizeof(data));

	return 0;
}

uint32_t Escritura_CRC(uint8_t origen, uint8_t destino, uint8_t size, uint8_t* payload_rx){

	uint8_t i=0;
	uint32_t CRC_;
    uint8_t relleno=0;
	uint8_t RESTOS=(11+size)%4;

	switch (RESTOS)
		{
			case 0:
				relleno=0;
			  break;
			case 1:
				relleno=3;
			  break;
			case 2:
				relleno=2;
			  break;
			case 3:
				relleno=1;
			  break;
		}

	uint8_t Data_Input [11+size+relleno];
	Data_Input[0] = 0xAA;
	Data_Input[1] = 0x55;
	Data_Input[2] = origen;
	Data_Input[3] = destino;
	Data_Input[4] = size;

	for (i=0 ; i<size ; i++)
	{
		Data_Input[5+i] = payload_rx[i];
	}

	Data_Input[5+size] = 0x00;
	Data_Input[6+size] = 0x00;
	Data_Input[7+size] = 0x00;
	Data_Input[8+size] = 0x00;
	Data_Input[9+size] = 0x55;
	Data_Input[10+size] = 0xAA;

	for (i=11+size ; i<=11+size+relleno ; i++)
		{
			Data_Input[i] = 0x00;
		}

	CRC_ = HAL_CRC_Calculate(&hcrc, (uint32_t*)Data_Input, sizeof(Data_Input) / sizeof(uint32_t));

	return(CRC_);
}

void Escritura_I2C(uint8_t *vec,uint8_t msg_length){

    uint8_t Transmision_I2C[2 + msg_length];
    uint8_t Recepcion_I2C[msg_length];
    uint8_t item= 0;

    for(item=0; item < (msg_length + 2);item++){
    	if(item == 0)
    		Transmision_I2C[item] = Direccion[0];
    	if(item == 1)
    		Transmision_I2C[item] = Direccion[1];
    	else Transmision_I2C[item] = vec[item-2];
    }
    Control_Byte = ( (CONTROL_CODE<<3)+BLOCK1 ) << 1;
	//Return = HAL_I2C_Master_Transmit(&hi2c1, Control_Byte,Data_I2C_Tx,8,HAL_MAX_DELAY);
	Return = HAL_I2C_Master_Transmit(&hi2c1, Control_Byte,Transmision_I2C,sizeof(Transmision_I2C),HAL_MAX_DELAY);
	HAL_Delay(10);
	Return = HAL_I2C_Master_Transmit(&hi2c1, Control_Byte,Direccion,2,HAL_MAX_DELAY);
	//HAL_I2C_Master_Receive(&hi2c1, Control_Byte,Data_I2C_Rx, sizeof(Transmision_I2C), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, Control_Byte,Recepcion_I2C, sizeof(Recepcion_I2C), HAL_MAX_DELAY);
}
//### Funcion que recibe por UART ###
uint8_t CSE_receive_data(){

	HAL_UART_Receive_IT(&huart6, &data_rx, sizeof(data_rx));
//HAL_UART_Receive_IT(huart, pData, Size)

	while(RX_PENDING);

	RX_PENDING = 1;

	return data_rx;
}
//### Completar esta función ###
void Enviar_Comando_AT(uint8_t* payload, uint8_t size_rx){

	uint8_t k=0;
	  uint8_t comando_AT[9] =  "AT+SEND+";
	  uint8_t size_AT = sizeof(comando_AT);
	  uint8_t mensaje_AT[size_AT+size_rx-1]; // el -1 es por el caracter nulo del vector char comando at
	for(k=0; k < size_AT-1; k++){
		mensaje_AT[k] =  comando_AT[k];
	}

	for(k=0; k < size_rx ;k++){
		mensaje_AT[k+size_AT-1] =  payload[k];
	}
	HAL_UART_Transmit_DMA(&huart2, mensaje_AT, sizeof(mensaje_AT));
	HAL_UART_Transmit_DMA(&huart1, mensaje_AT, sizeof(mensaje_AT));
}

uint8_t Run_Program(){

	int i;

		  uint8_t pos_START = 0; // Reseteo indice de posicion AA.

		  //HAL_UART_Receive_DMA(&huart2, Buffer_UART_Rx, sizeof(Buffer_UART_Rx)); // Recibo datos
		  //while(RX_PENDING);

	      pos_START = Encontrar_START(input_data,sizeof(input_data));

		  msg_length = 0;
		//  msg_length = contar_bytes_ciclicos(input_data, pos_START);

		  uint8_t payload[msg_length]; //inicializa

		  for(i=0; i < msg_length; i++){

			  *(payload + i) = *(input_data + pos_START + i + 2);

		  }//for

		  CSE_send_data(payload, dir_origen, dir_dest,msg_length);

		  //CSE_receive_data();
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart6)
{
	// Drivers > STM32F..HAL_Driver > Src > stm32f4xx_hal_uart.c

    RX_PENDING = 0;
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
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0); // RX =0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0); // Esta configurado en modo recepcion
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_MspInit(&huart2);
  HAL_UART_MspInit(&huart6);
  //Run_Program(); 				// Ejecución de programa.

  static uint8_t ESTADO = 0;
  //uint8_t size_rx = 0;

  uint8_t size_rx ;
  uint8_t payload_rx[256] = {0};

  uint8_t i , j , k;
  uint8_t CRC_rx[4];


  uint32_t crc_obt =0 ;
  uint32_t crc_aux =0 ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0); // configuro el max en modo recepcion
 HAL_Delay(1); // delay de 10ms para que tenga tiempo para poder recibir paquetes

 while (1)
  {
//	     Enviar_Comando_AT(payloadprueba,sizeof(payloadprueba)); // Para probar el envio de datos a la LORA
//		 CSE_send_data("OK", 0x06 , 0x05 , sizeof("OK")); // Para probar el envio de datos de OK
//		 CSE_send_data("ER", 0x06 , 0x05 , sizeof("ER")); // Para probar el envio de datos de ERROR
	  //CSE_send_data("OK", 0x06 , 0x05 , sizeof("OK"));
	 				 //Enviar_Comando_AT(payload_rx,size_rx);
	switch (ESTADO)
	{
		case ESTADO0:
			info[0] = CSE_receive_data();
		  if(info[0] == 0xAA)
			  ESTADO = ESTADO2;
		  else
			  ESTADO = ESTADO0;
		  break;

		case ESTADO2:
			info[1] = CSE_receive_data();
		  if(info[1] == 0x55)
			  ESTADO = ESTADO3;
		  else
			  ESTADO = ESTADO0;
		  break;

		case ESTADO3:
			info[2] = CSE_receive_data();
		  if(info[2] == 0x05)
			 ESTADO = ESTADO4;
		  else
			  ESTADO = ESTADO0;
		 break;

		case ESTADO4:
			info[3] = CSE_receive_data();
		  if(info[3] == 0x06)
			 ESTADO = ESTADO5;
		  else
			  ESTADO = ESTADO0;
		  break;

		case ESTADO5:
			info[4] = CSE_receive_data();
			 size_rx = info[4];

			 for(i=0; i < size_rx; i++ ){
				 info[5+i] = CSE_receive_data();
				 payload_rx[i] = info[5+i];
			 }
			 ESTADO = ESTADO6;
		 break;

		case ESTADO6:
		  //uint8_t CRC_rx[4];
		  for(j=0;j < 4; j++){
			  info[5 + size_rx + j] = CSE_receive_data();
			  CRC_rx[j] = info[5 + size_rx + j];
		  }
			 ESTADO = ESTADO7;
		 break;

		case ESTADO7:
		  info[5 + size_rx + 4] = CSE_receive_data();
		  if(info[5 + size_rx + 4] == 0x55)
			 ESTADO = ESTADO8;
		  else
			  ESTADO = ESTADO0;
		  break;

		case ESTADO8:
		  info[5 + size_rx + 4 + 1] = CSE_receive_data();
		  if(info[5 + size_rx + 4 + 1] == 0xAA)
			 ESTADO = ESTADO9;
		  else
			  ESTADO = ESTADO0;
		  break;

		case ESTADO9: // Calculo de CRC
			 crc_obt = Escritura_CRC(0x05, 0x06, size_rx, payload_rx);
			 crc_aux=0;
			 for (i=0 ; i<3 ; i++){
			 		  crc_aux = CRC_rx[i] | crc_aux;
			 		  crc_aux =  crc_aux << 8 ;
			 	  }
			 	  crc_aux = CRC_rx[3] | crc_aux;
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1); // configuro el max en modo transmision
			 HAL_Delay(1); // delay de 10ms para que tenga tiempo para poder transmitir paquetes

			 if(crc_obt == crc_aux)
			 {
				 CSE_send_data("OK", 0x06 , 0x05 , sizeof("OK"));
				 HAL_Delay(10); // delay de 50ms para que tenga tiempo para poder recibir paquetes
				 Enviar_Comando_AT(payload_rx,size_rx);
				 ESTADO=ESTADO0;
			 }
			 else
			 {
				 CSE_send_data("ER", 0x06 , 0x05 , sizeof("ER"));
				 HAL_Delay(1); // delay de 50ms para que tenga tiempo para poder recibir paquetes
				 ESTADO=ESTADO0;
			 }
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0); // configuro el max en modo recepcion
			 HAL_Delay(1); // delay de 50ms para que tenga tiempo para poder recibir paquetes
		break;
	}
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

