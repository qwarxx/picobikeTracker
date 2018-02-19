/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
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
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "minmea.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0; /* set to 1 after User Button interrupt  */
static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
static uint32_t rd_ptr;
char OK_FLAG = 0;
char STAT_FLAG = 0;
char ERR_FLAG = 0;
char CONN_FLAG = 0;
char data_string[75];
uint8_t pos = 0;
uint8_t loops = 0;
uint8_t rxString[MAXLEN];
struct minmea_sentence_rmc frame_rmc;
struct minmea_sentence_gga frame_gga;
struct minmea_sentence_gsa frame_gsa;
struct minmea_sentence_gsa frame_gsv;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles(void);
void send(char string[]);
void http_get(char data[]);
void gprs_connect(void);
static void msgrx_parse(void);
//void executeSerialCommand(uint8_t string[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	/* USER CODE BEGIN 2 */
	/* enable printing messages to console */
	initialise_monitor_handles();

	/* Start the receiver */
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	HAL_UART_Receive_DMA(&huart2, rx_dma_circ_buf, CIRC_BUF_SZ);
	rd_ptr = 0;

	printf("Init\n");

	HAL_Delay(1000);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	printf("Start the module!\n");
	send("\n\rAT+CGPSOUT=0\n\r");
	printf("[SEND] AT+CGPSOUT=0\n");
	send("\n\rAT\n\r");
	printf("[SEND] AT\n");

	send("\n\rAT+CGPSPWR=1\n\r");
	printf("[SEND] AT+CGPSPWR=1\n");
	//send("\n\rAT+CGPSRST=0\n\r");
	//printf("[SEND] AT+CGPSRST=0\n");
	send("\n\rAT+CGPSOUT=255\n\r");
	printf("[SEND] AT+CGPSOUT=255\n");
	HAL_Delay(500);
	msgrx_parse();

	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint8_t i = 0;
		HAL_Delay(2000);
		msgrx_parse();
		for (i = 0; i < frame_gga.satellites_tracked; i++) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_Delay(200);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		printf(
				"%02i%02i%02i.000,%09.4f,%010.4f,%.1f,%.1f,%i,%.2f,%.2f,%.2f,%02i%02i%02i,%02i\n",
				frame_rmc.time.hours, frame_rmc.time.minutes,
				frame_rmc.time.seconds, minmea_tofloat(&frame_rmc.latitude),
				minmea_tofloat(&frame_rmc.longitude),
				minmea_tofloat(&frame_gga.hdop),
				minmea_tofloat(&frame_gga.altitude), frame_gsa.fix_type,
				minmea_tofloat(&frame_rmc.course),
				minmea_tofloat(&frame_rmc.speed) * 1.852001,
				minmea_tofloat(&frame_rmc.speed), frame_rmc.date.day,
				frame_rmc.date.month, frame_rmc.date.year,
				frame_gga.satellites_tracked);
		sprintf(data_string,
				"%02i%02i%02i.000,%09.4f,%010.4f,%.1f,%.1f,%i,%.2f,%.2f,%.2f,%02i%02i%02i,%02i",
				frame_rmc.time.hours, frame_rmc.time.minutes,
				frame_rmc.time.seconds, minmea_tofloat(&frame_rmc.latitude),
				minmea_tofloat(&frame_rmc.longitude),
				minmea_tofloat(&frame_gga.hdop),
				minmea_tofloat(&frame_gga.altitude), frame_gsa.fix_type,
				minmea_tofloat(&frame_rmc.course),
				minmea_tofloat(&frame_rmc.speed) * 1.852001,
				minmea_tofloat(&frame_rmc.speed), frame_rmc.date.day,
				frame_rmc.date.month, frame_rmc.date.year,
				frame_gga.satellites_tracked);

		if (frame_gsa.fix_type == 3 || frame_gsa.fix_type == 1) {
			send("\n\rAT+CGPSOUT=0\n\r");
			printf("[SEND] AT+CGPSOUT=0\n");
			printf("[SEND] AT+SAPBR=2,1\n");
			send("AT+SAPBR=2,1\n\r");
			if (CONN_FLAG == 0 || CONN_FLAG == 3) {
				gprs_connect();
				CONN_FLAG = 0;
			}
			http_get(data_string);
			send("\n\rAT+CGPSOUT=255\n\r");
			printf("[SEND] AT+CGPSOUT=255\n");
		}
		if (loops > 600 && frame_gga.satellites_tracked < 1) {
			printf("[SEND] AT+CGPSRST=0\n");
			send("AT+CGPSRST=0\r\n");
		}
		loops++;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SIM808_PB_GPIO_Port, SIM808_PB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC1 PC2 PC3
	 PC4 PC5 PC6 PC7
	 PC8 PC9 PC10 PC11
	 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA6
	 PA7 PA8 PA9 PA10
	 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6
			| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
			| GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB11
	 PB12 PB13 PB14 PB15
	 PB5 PB6 PB7 PB8
	 PB9 */
	GPIO_InitStruct.Pin =
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SIM808_PWR_Pin */
	GPIO_InitStruct.Pin = SIM808_PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SIM808_PWR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SIM808_PB_Pin */
	GPIO_InitStruct.Pin = SIM808_PB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SIM808_PB_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static uint8_t msgrx_circ_buf_get(void) {
	uint8_t c = 0;
	if (rd_ptr != DMA_WRITE_PTR) {
		c = rx_dma_circ_buf[rd_ptr++];
		rd_ptr &= (CIRC_BUF_SZ - 1);
	}
	return c;
}

static void msgrx_parse(void) {

	uint8_t c = 0;
	uint16_t i = 0;
	uint16_t count = 0;

	//check if new data is in buffer
	while (count < CIRC_BUF_SZ + 1) {
		if (rd_ptr == DMA_WRITE_PTR) {
			break;
		}
		c = msgrx_circ_buf_get();
		//find beginning of message
		if (c == '\n') {
			c = msgrx_circ_buf_get();
			if (c == '$') {
				for (i = 0; i < MAXLEN; i++)
					rxString[i] = 0; // Clear the string buffer

				for (i = 0; i < MAXLEN; i++) {
					//detect end of message
					if (c == '\n') {
						rxString[i] = c;
						rxString[i + 1] = '\0';
						// detect type of message
						switch (minmea_sentence_id(rxString, false)) {
						case MINMEA_SENTENCE_RMC: {
							if (minmea_parse_rmc(&frame_rmc, rxString)) {

							}
							break;
						}
						case MINMEA_SENTENCE_GGA: {
							if (minmea_parse_gga(&frame_gga, rxString)) {

							}
							break;
						}
						case MINMEA_SENTENCE_GSA: {
							if (minmea_parse_gsa(&frame_gsa, rxString)) {

							}

							break;
						}
						case MINMEA_SENTENCE_GSV: {
							if (minmea_parse_gsa(&frame_gsv, rxString)) {

							}

							break;
						}
						}
						break;
					} else {
						rxString[i] = c;
					}
					c = msgrx_circ_buf_get();
				}

			} else if (c == '>') {
				OK_FLAG = 1;
			} else if (c == 'O') {
				c = msgrx_circ_buf_get();
				if (c == 'K') {
					OK_FLAG = 1;
				}

			} else if (c == 'E') {
				c = msgrx_circ_buf_get();
				if (c == 'R') {
					c = msgrx_circ_buf_get();
					if (c == 'R') {
						ERR_FLAG = 1;
					}
				}

			} else if (c == '+') {
				for (i = 0; i < 20; i++) {
					c = msgrx_circ_buf_get();
					if (c == ' ') {
						c = msgrx_circ_buf_get();
						if (c == '0') {
							STAT_FLAG = 0;
							break;
						} else if (c == '1') {
							STAT_FLAG = 1;
							c = msgrx_circ_buf_get();
							if (c == ',') {
								c = msgrx_circ_buf_get();
								if (c == '1') {
									CONN_FLAG = 1;
									break;
								} else {
									CONN_FLAG = 0;
									break;
								}
							}
							else{
								break;
							}
						}
						else{
							break;
						}
					}
				}
			}
		}
		count++;
	}
	__HAL_UART_FLUSH_DRREGISTER(&huart2);  //clear buffer after reading

}

/* Prints the supplied string to UART and checks for OK*/
void send(char string[]) {
	uint8_t i = 0;
	uint8_t try = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) string, strlen(string), 500);
	for (try = 0; try < 3; try++) {
		//retry sending 3 times
		OK_FLAG = 0;
		ERR_FLAG = 0;
		STAT_FLAG = 0;
		CONN_FLAG = 0;
		if (try == 2) {
			printf("[ERROR] No OK received\n");
		}
		for (i = 0; i < 500; i++) { //wait for 10 seconds for response before retrying
			msgrx_parse();
			HAL_Delay(20);
			if (OK_FLAG == 1 || CONN_FLAG != 0) {
				break;
			} else if (ERR_FLAG == 1) {
				printf("ERROR\n");
				break;
			}
		}
		if (OK_FLAG == 1 || STAT_FLAG == 1) {
			break;
		}
		HAL_UART_Transmit(&huart2, (uint8_t*) string, strlen(string), 500);

	}
	OK_FLAG = 0;
	ERR_FLAG = 0;
}

void gprs_connect(void) {
	uint8_t i = 0;
	send("\n\rAT+CSTT=pinternet.interkom.de\n\r");
	printf("[SEND] AT+CSTT=pinternet.interkom.de\n");
	send("\n\rAT+CIICR\n\r");
	printf("[SEND] AT+CIICR\n");
	send("\n\rAT+CGATT?\n\r");
	printf("[SEND] AT+CGATT?\n");
	if (STAT_FLAG != 1) {
		send("\n\rAT+CFUN=1,1\n\r");
		printf("[SEND] AT+CFUN=1,1\n");
		HAL_Delay(3000);

		send("\n\rAT+CIICR\n\r");
		printf("[SEND] AT+CIICR\n");
	}

	printf("[SEND] AT+CGATT?\n");
	send("\n\rAT+CGATT?\n\r");
	if (STAT_FLAG != 1) {
		printf("[SEND] AT+CGATT=1\n");

		send("AT+CGATT=1\n\r");

	}
	STAT_FLAG = 0;
	printf("[SEND] AT+CFUN?\n");
	send("AT+CFUN?\n\r");
	if (STAT_FLAG != 1) {
		printf("[SEND] AT+CFUN=1,1\n");
		send("AT+CFUN=1,1\n\r");
	}
	STAT_FLAG = 0;

	for (i = 0; i < 3; i++) {
		printf("[SEND] AT+SAPBR=2,1\n");
		send("AT+SAPBR=2,1\n\r");
		if (CONN_FLAG != 1) {
			send("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\n\n\r");
			printf("[SEND] AT+SAPBR=3,1,\"Contype\",\"GPRS\"\n");
			send("AT+SAPBR=3,1,\"APN\",\"pinternet.interkom.de\"\n\r");
			printf("[SEND] AT+SAPBR=3,1,\"APN\",\"pinternet.interkom.de\"\n");
			send("AT+SAPBR=1,1\n\r");
			printf("[SEND] AT+SAPBR=1,1\n");
		} else if (CONN_FLAG == 1) {
			CONN_FLAG = 0;

			send("AT+HTTPINIT\n\r");
			printf("[SEND] AT+HTTPINIT\n");
			send("AT+HTTPPARA=\"CID\",1\n\r");
			printf("[SEND] AT+HTTPPARA=\"CID\",1\n");
			break;
		}
	}
	send("AT+HTTPINIT\n\r");
	printf("[SEND] AT+HTTPINIT\n");
	send("AT+HTTPPARA=\"CID\",1\n\r");
	printf("[SEND] AT+HTTPPARA=\"CID\",1\n");
	HAL_Delay(2000);

	STAT_FLAG = 0;
	CONN_FLAG = 0;
}

void http_get(char data[]) {
	char string[strlen(data) + 100];
	sprintf(string,
			"AT+HTTPPARA=\"URL\",\"http://updates.opengps.net/index.php?imei=865067025975296&key=beanbean100&data=%s\"\n\r",
			data);
	HAL_Delay(1000);
	send(string);
	HAL_Delay(1000);
	printf(string);
	HAL_Delay(3000);
	send("AT+HTTPACTION=0\n\r");
	HAL_Delay(1000);
	printf("AT+HTTPACTION=0\n");
	HAL_Delay(500);
	if (CONN_FLAG != 1) {
		printf("[SEND] AT+SAPBR=0,1\n");
		send("AT+SAPBR=0,1\n\r");
		HAL_Delay(500);
		printf("[SEND] AT+SAPBR=1,1\n");
		send("AT+SAPBR=1,1\n\r");
		HAL_Delay(500);
	}
}

/**
 * @brief EXTI line detection callbacksp
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		UserButtonStatus = 1;
	}
}

/* UART TX complete callback */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {

}

/* UART RX complete callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	msgrx_parse();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("\r\n[ERROR] Unspecified error");
	while (1) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(500);
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong 	OK_FLAG = 0;
	 parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
