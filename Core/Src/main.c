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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#define CAN_QUEUE_SIZE 32

#define MAX_CAN_ID 350

typedef struct {
	uint32_t id;
	uint64_t data; // 8‑byte payload for CAN Classic frames
} can_data_t;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WS2812_LED_COUNT 1
#define WS2812_BITS_PER_LED 24
#define WS2812_RESET_SLOTS 100
#define WS2812_BUFFER_SIZE (WS2812_LED_COUNT * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

#define WS2812_DUTY_0 12
#define WS2812_DUTY_1 22

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//FDCAN_FilterTypeDef sFilterConfig;
//
//FDCAN_RxHeaderTypeDef RxHeader;
//FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

FDCAN_FilterTypeDef sFilterConfig;

uint8_t RxData[8];

HAL_StatusTypeDef status;

uint8_t TxData[8];
uint8_t TxData_loopback[8];
uint8_t RxData_loopback[8];

int16_t imu[3];

uint16_t id;

uint16_t dados[WS2812_BUFFER_SIZE];
uint16_t c1, c2, c3;

uint16_t timestamp_us;

volatile uint32_t systemMs = 0;

can_data_t callbk_can_buff[100];
char csv_line[2048];

int16_t can_id_to_index[MAX_CAN_ID + 1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart4, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

int fputc(int ch, FILE *f) {
	return __io_putchar(ch);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t pwmBusy = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
		pwmBusy = 0;
	}
}

void ws2812_show(void) {
	while (pwmBusy)
		;

	pwmBusy = 1;

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

	if (HAL_TIM_PWM_Start_DMA(&htim2,
	TIM_CHANNEL_3, (uint32_t*) dados, WS2812_BUFFER_SIZE) != HAL_OK) {
		pwmBusy = 0;
	}
}

void ws2812_stop(void) {
	while (pwmBusy)
		;

	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	pwmBusy = 0;
}

static void ws2812_write_byte(uint8_t value, uint16_t *buffer) {
	for (int bit = 7; bit >= 0; bit--) {
		*buffer++ = (value & (1 << bit)) ? WS2812_DUTY_1 : WS2812_DUTY_0;
	}
}

void led_rgb(uint16_t red, uint16_t green, uint16_t blue) {
	uint16_t *buffer = dados;

	ws2812_write_byte((uint8_t) green, buffer);
	buffer += 8;
	ws2812_write_byte((uint8_t) red, buffer);
	buffer += 8;
	ws2812_write_byte((uint8_t) blue, buffer);

	for (int i = WS2812_BITS_PER_LED; i < WS2812_BUFFER_SIZE; i++) {
		dados[i] = 0;
	}

	ws2812_show();
}

void ws2812_off(void) {
	led_rgb(0, 0, 0);
}

void blink_rgb(void) {
	led_rgb(120, 0, 0);
	HAL_Delay(200);
	led_rgb(0, 120, 0);
	HAL_Delay(200);
	led_rgb(0, 0, 120);
	HAL_Delay(200);
	ws2812_off();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	uint32_t ts = __HAL_TIM_GET_COUNTER(&htim3);
	HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData);
	int16_t index = -1;
	if (RxHeader.Identifier <= MAX_CAN_ID) {
		index = can_id_to_index[RxHeader.Identifier];
		if (index >= 0 && index < 100) {
			memcpy(&callbk_can_buff[index].data, RxData, sizeof(uint64_t));
			callbk_can_buff[index].id = RxHeader.Identifier;
		}
	}
#ifdef DEBUG_UART
    char dbg[64];
    snprintf(dbg, sizeof(dbg), "%lu us ID=0x%03lX\r\n", ts, RxHeader.Identifier);
    debug_uart_puts(dbg);
    #endif
	// Keep printf for development if needed
	printf("%lu us ID=0x%03lX\r\n", ts, RxHeader.Identifier);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_USART1_UART_Init();
	MX_FATFS_Init();
	MX_UART4_Init();
	MX_SDMMC2_SD_Init();
	MX_FDCAN2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
	HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(SDMMC2_IRQn);

//	if (HAL_SD_ConfigWideBusOperation(&hsd2,
//	SDMMC_BUS_WIDE_4B) != HAL_OK) {
//		printf("Erro bus width\r\n");
//	}

	HAL_SD_CardInfoTypeDef info;

	if (HAL_SD_GetCardInfo(&hsd2, &info) == HAL_OK) {
		printf("SD OK\r\n");
	} else {
		printf("SD FAIL\r\n");
	}

	FATFS meuFATFS;
	FIL meuArquivo;
	UINT testeByte;

	//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, dados, 74);

	// Mount filesystem
	FRESULT mount_res = f_mount(&meuFATFS, SDPath, 1);
	if (mount_res != FR_OK) {
		printf("Falha ao montar Logical driver do Cartao sd\r\n");
		Error_Handler();
	}

	// Initialize buffer mapping – uncomment and set needed IDs
	// Example mappings (adjust as needed):
	// can_id_to_index[259] = 0;   // variable 1
	// can_id_to_index[260] = 1;   // variable 2
	// can_id_to_index[261] = 2;   // variable 3
	// ... add other mappings up to 99 entries ...

	// Open (or create) log file in append mode
	FRESULT open_res = f_open(&meuArquivo, "Arquivo.txt",
	FA_OPEN_APPEND | FA_WRITE);
	if (open_res == FR_OK) {
		// If file is empty, write header
		if (f_size(&meuArquivo) == 0) {
			const char *header =
					"TIMESTAMP,GYROSCOPE,ACCELEROMETER,YAW_INPUT,RESERVED,RESERVED,RESERVED,RESERVED,STATUS_AIR + GLV + SHUNT,TOTAL_VOLTAGE_STACK1-3,TOTAL_VOLTAGE_STACK4-6,ACCUMULATOR INFORMATION,TIMESTAMP_GNSS,LATITUDE,LONGITUDE,GENERAL_GNSS,DRIVER_INPUTS,REF_TORQUE,WHEEL_SPEED,WHEEL_ACCEL,LEFT_MOTOR_TRACTIVE,LEFT_MOTOR_CURRENT,LEFT_MOTOR_ENERGY_TEMP,LEFT_MOTOR_COMMUNICATION,LEFT_MOTOR_STATE,RIGHT_MOTOR_TRACTIVE,RIGHT_MOTOR_CURRENT,RIGHT_MOTOR_ENERGY_TEMP,RIGHT_MOTOR_COMMUNICATION,RIGHT_MOTOR_STATE,RESERVED,RESERVED,RESERVED,RESERVED,STACK_VOLTAGE_1,STACK_VOLTAGE_2,STACK_VOLTAGE_3,STACK_VOLTAGE_4,STACK_VOLTAGE_5,STACK_VOLTAGE_6,STACK_TEMPERATURE_1,STACK_TEMPERATURE_2,STACK_TEMPERATURE_3,STACK_TEMPERATURE_4,STACK_TEMPERATURE_5,STACK_TEMPERATURE_6,BMS_ERROR_LOG_1,BMS_ERROR_LOG_2,ERROR_SUM_SLAVE_1,ERROR_SUM_SLAVE_2,BMS MODE,ERROR_MODE_STACK_VOLTAGE_1,ERROR_MODE_STACK_VOLTAGE_2,ERROR_MODE_STACK_VOLTAGE_3,ERROR_MODE_STACK_VOLTAGE_4,ERROR_MODE_STACK_VOLTAGE_5,ERROR_MODE_STACK_VOLTAGE_6,ERROR_MODE_STACK_TEMPERATURE_1,ERROR_MODE_STACK_TEMPERATURE_2,ERROR_MODE_STACK_TEMPERATURE_3,ERROR_MODE_STACK_TEMPERATURE_4,ERROR_MODE_STACK_TEMPERATURE_5,ERROR_MODE_STACK_TEMPERATURE_6,ADDR VOLT MIN,ADDR VOLT MAX,ADDR MAX TEMP,DESIRED YAW,ECU_MODE,TORQUE_GAIN,CONTROL_EVENTS,HODOMETER,SET_POINT,SLIP_RATE,NTC1-3,NTC4-6,NTC7-9,MLX1,MLX2,ELETROBUILD_TEMPERATURE\r\n";
			UINT bw;
			f_write(&meuArquivo, header, strlen(header), &bw);
		}
	} else {
		printf("Falha ao abrir/criar arquivo log\r\n");
		Error_Handler();
	}

	// Timestamp baseline for logging interval
	uint32_t timestamp_base = __HAL_TIM_GET_COUNTER(&htim3);
	// Initialize CAN buffer status
	memset(callbk_can_buff, 0, sizeof(callbk_can_buff));
	memset(can_id_to_index, -1, sizeof(can_id_to_index));

	printf("Arquivo existente\r\n");

	blink_rgb();

	FRESULT write_res;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		static uint32_t last_uart_heartbeat = 0;

		if (HAL_GetTick() - last_uart_heartbeat >= 100) {
			last_uart_heartbeat = HAL_GetTick();
			printf("tick %lu\r\n", HAL_GetTick());
		}

		char *ptr = csv_line;

		ptr += sprintf(ptr, "%lu", HAL_GetTick());

		for (int i = 0; i < 82; i++) {
			// Use 64-bit format specifier for CAN data payload
			ptr += sprintf(ptr, ",%llu",
					(unsigned long long) callbk_can_buff[i].data);
		}

		ptr += sprintf(ptr, "\r\n");

		// Write CSV line to SD card
		write_res = f_write(&meuArquivo, csv_line, strlen(csv_line),
				&testeByte);
		if (write_res != FR_OK) {
			printf("SD write error: %d\r\n", write_res);
		} else {
			printf("escrita realizada!\r\n");
			// Ensure data is flushed to card periodically
			f_sync(&meuArquivo);
		}
		// Do NOT close the file each iteration; keep it open for continuous logging

//				res = f_mount(NULL, SDPath, 1);
//			} else {
//				printf("Falha ao montar Logical driver do Cartao sd \r\n");
//			}
//		}

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 10;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = ENABLE;
	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;
	hfdcan2.Init.NominalPrescaler = 2;
	hfdcan2.Init.NominalSyncJumpWidth = 1;
	hfdcan2.Init.NominalTimeSeg1 = 13;
	hfdcan2.Init.NominalTimeSeg2 = 2;
	hfdcan2.Init.DataPrescaler = 1;
	hfdcan2.Init.DataSyncJumpWidth = 1;
	hfdcan2.Init.DataTimeSeg1 = 1;
	hfdcan2.Init.DataTimeSeg2 = 1;
	hfdcan2.Init.MessageRAMOffset = 0;
	hfdcan2.Init.StdFiltersNbr = 1;
	hfdcan2.Init.ExtFiltersNbr = 1;
	hfdcan2.Init.RxFifo0ElmtsNbr = 8;
	hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxFifo1ElmtsNbr = 8;
	hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxBuffersNbr = 1;
	hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.TxEventsNbr = 1;
	hfdcan2.Init.TxBuffersNbr = 1;
	hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;

	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;

	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

	HAL_FDCAN_Start(&hfdcan2);

	HAL_FDCAN_ActivateNotification(&hfdcan2,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	/* USER CODE END FDCAN2_Init 2 */

}

/**
 * @brief SDMMC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC2_SD_Init(void) {

	/* USER CODE BEGIN SDMMC2_Init 0 */

	/* USER CODE END SDMMC2_Init 0 */

	/* USER CODE BEGIN SDMMC2_Init 1 */

	/* USER CODE END SDMMC2_Init 1 */
	hsd2.Instance = SDMMC2;
	hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd2.Init.ClockDiv = 64;
	/* USER CODE BEGIN SDMMC2_Init 2 */

	uint8_t sd_ok = 0;

	for (int tentativas = 0; tentativas < 10; tentativas++) {
		if (HAL_SD_Init(&hsd2) == HAL_OK) {
			sd_ok = 1;
			printf("SD inicializado na tentativa %d\r\n", tentativas + 1);
			break;
		}

		printf("Falha SD tentativa %d, erro=0x%08lx\r\n", tentativas + 1,
				hsd2.ErrorCode);

		HAL_SD_DeInit(&hsd2);
		HAL_Delay(500);
	}

	if (!sd_ok) {
		printf("Nao foi possivel inicializar SD\r\n");

		Error_Handler();
		NVIC_SystemReset();
	}

	/* USER CODE END SDMMC2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 39;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 63;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 50000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Debug_SD_GPIO_Port, Debug_SD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : Debug_SD_Pin */
	GPIO_InitStruct.Pin = Debug_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Debug_SD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	if (htim->Instance == TIM3) {
		systemMs++;
	}

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
