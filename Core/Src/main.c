/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "radio_driver.h"
#include "stm32wlxx_nucleo.h" // Gives LED helpers

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	MASTER,
	SLAVE
} State;

typedef enum{
	RX,
	TX
} SubState;

typedef struct{
	State state;
	SubState subState;
	uint32_t rxTimeout, txDelay;
	uint8_t rxLen;
	char rxBuffer[RX_BUFFER_SIZE];
} SessionContext;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MAX_BUFFER_SIZE 		255
#define RF_FREQ		 			868000000 	// Hz = 868 MHz - Ranges in PK {433.05 - 434.79}, {865 - 869}, {920 - 925}
#define TX_POWER		 		14        	// dBm
#define LORA_BANDWIDTH 			LORA_BW_125	// kHz
#define LORA_SPREADING_FACTOR 	7		  	// SF7
#define LORA_CODING_RATE 		1		  	// 4/5
#define LORA_PREAMBLE_LENGTH 	8         	// Same for Tx and Rx

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t buffer[MAX_BUFFER_SIZE], output[MAX_BUFFER_SIZE], input[1];
uint16_t count = 0;
bool messageReady = false;
char id[MAX_BUFFER_SIZE] = "\r\nSetting your ID as";
int idLen = -2;

void (*volatile currentEvent)(SessionContext*);
PacketParams_t packetParams;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UART_Transmit(const char*);
void resetTerminal();
void interruptTerminal(const char*);

void Radio_Init();
void Radio_DIO_IRq_Callback_Handler(const RadioIrqMasks_t);
void rxErrorEvent(SessionContext*);

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

  HAL_Delay(3000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_BLUE);	// Master
  BSP_LED_Init(LED_GREEN);	// Slave
  BSP_LED_Init(LED_RED);	// Disconnected

  Radio_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  snprintf((char*)buffer, MAX_BUFFER_SIZE, "\r\nSTM32 SubGHz LoRa Messenger\r\n\r\nPlease Enter an ID: ");
  UART_Transmit((char*)buffer);
  while(!messageReady) HAL_UART_Receive_IT(&huart2, input, 1);
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  idLen = snprintf(id, MAX_BUFFER_SIZE, "%s", (char*)output);
  UART_Transmit(id);
  UART_Transmit("\r\n\r\n");
  resetTerminal();

  BSP_LED_On(LED_RED); // Disconnected at first

  SessionContext sessionContext = {
		  .state = MASTER,		// Start as Master
		  .subState = RX,		// Start by listening
		  .rxTimeout = 3000,	// ms
		  .txDelay = 100 		// ms
  };
  uint16_t mask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR;
  SUBGRF_SetDioIrqParams( mask, mask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );	// Arm radio IRQs for RX done, timeout, CRC error
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX); 								// Set RF switch to RX path on low-power PA path
  SUBGRF_SetRx(sessionContext.rxTimeout << 6); 							// SetRx(timeout): SX126x timeout units are 15.625 µs (1/64 ms). Multiplying ms by 64 = << 6.

  HAL_NVIC_EnableIRQ(USART2_IRQn);
  messageReady = false;

  while (1)
  {
    /* USER CODE END WHILE */

	currentEvent = NULL;
	while(!currentEvent) HAL_UART_Receive_IT(&huart2, input, 1);
	currentEvent(&sessionContext);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 👆🏼
 *  Enables backup domain access, sets LSE drive, uses MSI as SYSCLK, no PLL;
 *  APB clocks = AHB = SYSCLK;
 *  flash latency 2.
 *  This is a low-power, simple config suitable for the WL.
 */

void UART_Transmit(const char* string){
	HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void resetTerminal(){
	count = (uint16_t)snprintf((char*)buffer, MAX_BUFFER_SIZE, "%s: ", id);
	UART_Transmit((char*)buffer);
}

void interruptTerminal(const char* interruption){
	for(uint16_t x = 0; x < count; x++) UART_Transmit("\b \b");
	UART_Transmit("\r\n");
	UART_Transmit(interruption);
	UART_Transmit("\r\n\r\n");
	HAL_UART_Transmit(&huart2, buffer, count, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	switch(input[0]){
		case 0xd:
			UART_Transmit("\r\n");
			sprintf((char*)output, "%s", (char*)buffer);
			output[count] = '\0';
			messageReady = true;
			resetTerminal();
			break;

		case 0x8:
			if(count > (idLen + 2)){
				count--;
				UART_Transmit("\b \b");
			}
			break;

		default:
			if(count < MAX_BUFFER_SIZE){
				buffer[count++] = input[0];
				HAL_UART_Transmit(huart, input, 1, HAL_MAX_DELAY);
			}
	}
}

void Radio_DIO_IRq_Callback_Handler(const RadioIrqMasks_t radioIRq){
	switch(radioIRq){
		case IRQ_TX_DONE:
			interruptTerminal("\r\nTX DONE\r\n");
			break;

		case IRQ_RX_DONE:
			interruptTerminal("\r\nRX DONE\r\n");
			break;

		case IRQ_RX_TX_TIMEOUT:
			switch(SUBGRF_GetOperatingMode()){
				case MODE_TX: // TX Timeout
					interruptTerminal("\r\nTX TIMEOUT\r\n");
					break;

				case MODE_RX:	// RX Timeout
					interruptTerminal("\r\nRX TIMEOUT\r\n");
					currentEvent = rxErrorEvent;

					break;
				default:break;
			}
			break;

		case IRQ_CRC_ERROR: // Rx Error
			interruptTerminal("\r\nRX CRC ERROR\r\n");

			break;
		default: break;
	}
}

/** Initialize the Sub-GHz radio and dependent hardware.
  */
void Radio_Init(){
	// Initialize the hardware (SPI bus, TCXO control, RF switch) or the SUBGHZ (SX126x) and registers the IRQ callback.
	SUBGRF_Init(Radio_DIO_IRq_Callback_Handler);

	// Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
	// "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
	SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
	SUBGRF_SetRegulatorMode(); // use DCDC if configured in radio_conf.h

	// Use the whole 256-byte buffer for both TX and RX (starting at 0)
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);

	SUBGRF_SetRfFrequency(RF_FREQ);
	SUBGRF_SetRfTxPower(TX_POWER);
	SUBGRF_SetStopRxTimerOnPreambleDetect(false);

	SUBGRF_SetPacketType(PACKET_TYPE_LORA);

	// Sets LoRa private syncword (not the public 0x34). Ensures you only talk to your nodes (not public network).
	SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
	SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

	// Applies SF/BW/CR. Low data rate optimize off (OK for SF7/BW125).
	ModulationParams_t modulationParams;
	modulationParams.PacketType = PACKET_TYPE_LORA;
	modulationParams.Params.LoRa.Bandwidth = LORA_BANDWIDTH;
	modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODING_RATE;
	modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
	modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
	SUBGRF_SetModulationParams(&modulationParams);

	// CRC on, variable length, normal IQ, long RX FIFO length.
	packetParams.PacketType = PACKET_TYPE_LORA;
	packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
	packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
	packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
	SUBGRF_SetPacketParams(&packetParams);

	// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
	// RegIqPolaritySetup @address 0x0736
	// SX126x errata: improves IQ handling (safe even with normal IQ).
	SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}

/** MASTER/RX CRC/header error → treat like “no valid frame” and attempt TX "PING" after random backoff.
  * SLAVE/RX → simply re-enter RX.
  */
void rxErrorEvent(SessionContext *sessionContext){
	switch (sessionContext->state){
		case MASTER:
			interruptTerminal("Entering TX mode");
			sessionContext->subState = TX;
			break;

		case SLAVE:
			interruptTerminal("Entering RX mode");
			break;
		default:break;
	}
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
#ifdef USE_FULL_ASSERT
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
