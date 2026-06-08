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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define CAN_QUEUE_SIZE 32

typedef struct {
	uint16_t timestamp;
	uint32_t id;
	uint64_t data;

} can_data_t;

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

FDCAN_HandleTypeDef hfdcan2;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
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

uint16_t dados[74];
uint16_t c1, c2, c3;

uint16_t timestamp_us;

volatile uint32_t systemMs = 0;

SemaphoreHandle_t sd_write_sem;
QueueHandle_t can_data_queue;

UBaseType_t can_queue_len = 100;

can_data_t callbk_can_buff[100];

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
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *f) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t pwmBusy = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
	pwmBusy = 0;
}

void ws2812_show(void) {
	while (pwmBusy)
		;

	pwmBusy = 1;

	HAL_TIM_PWM_Start_DMA(&htim2,
	TIM_CHANNEL_3, (uint32_t*) dados, 74);
}

void led_rgb(uint16_t red, uint16_t green, uint16_t blue) {
	extern uint16_t dados[74];

	uint16_t greenbuffer[8];
	uint16_t redbuffer[8];
	uint16_t bluebuffer[8];

	uint16_t bit0 = 12;
	uint16_t bit1 = 22;

	/* GREEN */
	for (int i = 7; i >= 0; i--) {
		greenbuffer[i] = green % 2;
		green = green / 2;

		if (greenbuffer[i] == 1) {
			dados[i] = bit1;
		} else {
			dados[i] = bit0;
		}
	}

	/* RED */
	for (int i = 7; i >= 0; i--) {
		redbuffer[i] = red % 2;
		red = red / 2;

		if (redbuffer[i] == 1) {
			dados[i + 8] = bit1;
		} else {
			dados[i + 8] = bit0;
		}
	}

	/* BLUE */
	for (int i = 7; i >= 0; i--) {
		bluebuffer[i] = blue % 2;
		blue = blue / 2;

		if (bluebuffer[i] == 1) {
			dados[i + 16] = bit1;
		} else {
			dados[i + 16] = bit0;
		}
	}

	for (int i = 24; i < 74; i++) {
		dados[i] = 0;
	}

	ws2812_show();
}

void blink_rgb(void) {
	led_rgb(0, 0, 0);
	HAL_Delay(200);
	led_rgb(120, 0, 0);
	HAL_Delay(200);
	led_rgb(0, 120, 0);
	led_rgb(0, 0, 0);
	HAL_Delay(200);
	led_rgb(0, 0, 120);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	uint32_t ts;

	ts = __HAL_TIM_GET_COUNTER(&htim3);

	HAL_FDCAN_GetRxMessage(&hfdcan2,
	FDCAN_RX_FIFO0, &RxHeader, RxData);

	uint32_t index = RxHeader.Identifier - 1;

	if (index >= 0 && index <= 81) {
		// Copia os 8 bytes diretamente para o uint64_t data
		memcpy(&callbk_can_buff[index].data, RxData, 8);
		callbk_can_buff[index].timestamp = HAL_GetTick();
		callbk_can_buff[index].id = RxHeader.Identifier;
	}

	printf("%lu us ID=0x%03lX\r\n", ts, RxHeader.Identifier);

}

void Periodic_SD_save_task(void *pvParameters) {
	while (1) {
		FATFS meuFATFS;
		FIL meuArquivo;
		UINT testeByte;

//		xSemaphoreTake(sd_write_sem, portMAX_DELAY);

		char line[1024] = { 0 };

		FRESULT res = f_mount(&meuFATFS, SDPath, 1);
		if (res == FR_OK) {

			res = f_open(&meuArquivo, "Arquivo.txt",
			FA_WRITE | FA_CREATE_ALWAYS);

			char csv_line[2048];

			callbk_can_buff[0].data = 234;

//			snprintf(csv_line, sizeof(csv_line),
//			// HAL_GetTick()
//					"%lu,"
//					// 0-6
//							"%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 7-14
//							"%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 15-23
//							"%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 24-32
//							"%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 33-38
//							"%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 39-44
//							"%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 45-49
//							"%llu,%llu,%llu,%llu,%llu,"
//							// 50-55
//							"%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 56-61
//							"%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 62-65
//							"%llu,%llu,%llu,%llu,"
//							// 66-71
//							"%llu,%llu,%llu,%llu,%llu,%llu,"
//							// 72-76
//							"%llu,%llu,%llu,%llu,%llu,"
//							// 77-81
//							"%llu,%llu,%llu,%llu,%llu\r\n",
//
//					HAL_GetTick(),
//
//					/* 0 - 6 */
//					callbk_can_buff[0].data,     // 1 GYROSCOPE
//					callbk_can_buff[1].data,     // 2 ACCELEROMETER
//					callbk_can_buff[2].data,     // 3 YAW_INPUT
//					callbk_can_buff[3].data,     // 4 RESERVED
//					callbk_can_buff[4].data,     // 5 RESERVED
//					callbk_can_buff[5].data,     // 6 RESERVED
//					callbk_can_buff[6].data,     // 7 RESERVED
//
//					/* 7 - 14 */
//					callbk_can_buff[7].data,     // 8 STATUS_AIR + GLV + SHUNT
//					callbk_can_buff[8].data,     // 9 TOTAL_VOLTAGE_STACK1-4
//					callbk_can_buff[9].data, // 10 TOTAL_VOLTAGE_STACK4-6 + MIN_VOLT
//					callbk_can_buff[10].data,    // 11 MAX_VOLT + MAX_TEMP
//					callbk_can_buff[11].data,    // 12 RESERVED
//					callbk_can_buff[12].data,    // 13 RESERVED
//					callbk_can_buff[13].data,    // 14 RESERVED
//					callbk_can_buff[14].data,    // 15 RESERVED
//
//					/* 15 - 23 */
//					callbk_can_buff[15].data,    // 16 DRIVER_INPUTS
//					callbk_can_buff[16].data,    // 17 REF_TORQUE
//					callbk_can_buff[17].data,    // 18 WHEEL_SPEED
//					callbk_can_buff[18].data,    // 19 WHEEL_ACCEL
//					callbk_can_buff[19].data,    // 20 LEFT_MOTOR_TRACTIVE
//					callbk_can_buff[20].data,    // 21 LEFT_MOTOR_CURRENT
//					callbk_can_buff[21].data,    // 22 LEFT_MOTOR_ENERGY_TEMP
//					callbk_can_buff[22].data,    // 23 LEFT_MOTOR_COMMUNICATION
//					callbk_can_buff[23].data,    // 24 LEFT_MOTOR_STATE
//
//					/* 24 - 32 */
//					callbk_can_buff[24].data,    // 25 RIGHT_MOTOR_TRACTIVE
//					callbk_can_buff[25].data,    // 26 RIGHT_MOTOR_CURRENT
//					callbk_can_buff[26].data,    // 27 RIGHT_MOTOR_ENERGY_TEMP
//					callbk_can_buff[27].data,    // 28 RIGHT_MOTOR_COMMUNICATION
//					callbk_can_buff[28].data,    // 29 RIGHT_MOTOR_STATE
//					callbk_can_buff[29].data,    // 30 RESERVED
//					callbk_can_buff[30].data,    // 31 RESERVED
//					callbk_can_buff[31].data,    // 32 RESERVED
//					callbk_can_buff[32].data,    // 33 RESERVED
//
//					/* 33 - 38 */
//					callbk_can_buff[33].data,    // 34 STACK_VOLTAGE
//					callbk_can_buff[34].data,    // 35 STACK_VOLTAGE
//					callbk_can_buff[35].data,    // 36 STACK_VOLTAGE
//					callbk_can_buff[36].data,    // 37 STACK_VOLTAGE
//					callbk_can_buff[37].data,    // 38 STACK_VOLTAGE
//					callbk_can_buff[38].data,    // 39 STACK_VOLTAGE
//
//					/* 39 - 44 */
//					callbk_can_buff[39].data,    // 40 STACK_TEMPERATURE
//					callbk_can_buff[40].data,    // 41 STACK_TEMPERATURE
//					callbk_can_buff[41].data,    // 42 STACK_TEMPERATURE
//					callbk_can_buff[42].data,    // 43 STACK_TEMPERATURE
//					callbk_can_buff[43].data,    // 44 STACK_TEMPERATURE
//					callbk_can_buff[44].data,    // 45 STACK_TEMPERATURE
//
//					/* 45 - 49 */
//					callbk_can_buff[45].data,    // 46 GENERAL_BMS
//					callbk_can_buff[46].data,    // 47 BMS_ERROR_LOG
//					callbk_can_buff[47].data,    // 48 BMS_ERROR_LOG
//					callbk_can_buff[48].data,    // 49 ERROR_SUM_SLAVE
//					callbk_can_buff[49].data,   // 50 ERROR_SUM_SLAVE + BMS_MODE
//
//					/* 50 - 55 */
//					callbk_can_buff[50].data,    // 51 ERROR_MODE_STACK_VOLTAGE
//					callbk_can_buff[51].data,    // 52 ERROR_MODE_STACK_VOLTAGE
//					callbk_can_buff[52].data,    // 53 ERROR_MODE_STACK_VOLTAGE
//					callbk_can_buff[53].data,    // 54 ERROR_MODE_STACK_VOLTAGE
//					callbk_can_buff[54].data,    // 55 ERROR_MODE_STACK_VOLTAGE
//					callbk_can_buff[55].data,    // 56 ERROR_MODE_STACK_VOLTAGE
//
//					/* 56 - 61 */
//					callbk_can_buff[56].data, // 57 ERROR_MODE_STACK_TEMPERATURE
//					callbk_can_buff[57].data, // 58 ERROR_MODE_STACK_TEMPERATURE
//					callbk_can_buff[58].data, // 59 ERROR_MODE_STACK_TEMPERATURE
//					callbk_can_buff[59].data, // 60 ERROR_MODE_STACK_TEMPERATURE
//					callbk_can_buff[60].data, // 61 ERROR_MODE_STACK_TEMPERATURE
//					callbk_can_buff[61].data, // 62 ERROR_MODE_STACK_TEMPERATURE
//
//					/* 62 - 65 */
//					callbk_can_buff[62].data,    // 63 RESERVED
//					callbk_can_buff[63].data,    // 64 RESERVED
//					callbk_can_buff[64].data,    // 65 RESERVED
//					callbk_can_buff[65].data,    // 66 RESERVED
//
//					/* 66 - 71 */
//					callbk_can_buff[66].data,    // 67 ECU_MODE
//					callbk_can_buff[67].data,    // 68 TORQUE_GAIN
//					callbk_can_buff[68].data,    // 69 CONTROL_EVENTS
//					callbk_can_buff[69].data,    // 70 HODOMETER
//					callbk_can_buff[70].data,    // 71 SET_POINT
//					callbk_can_buff[71].data,    // 72 SLIP_RATE
//
//					/* 72 - 76 */
//					callbk_can_buff[72].data,    // 73 NTC1-3
//					callbk_can_buff[73].data,    // 74 NTC4-6
//					callbk_can_buff[74].data,    // 75 NTC7-9
//					callbk_can_buff[75].data,    // 76 MLX1
//					callbk_can_buff[76].data,    // 77 MLX2
//
//					/* 77 - 81 */
//					callbk_can_buff[77].data,    // 78 TIMESTAMP_GNSS
//					callbk_can_buff[78].data,    // 79 LATITUDE
//					callbk_can_buff[79].data,    // 80 LONGITUDE
//					callbk_can_buff[80].data,    // 81 GENERAL_GNSS
//					callbk_can_buff[81].data     // 82 ELETROBUILD_TEMPERATURE
//					);
			res = f_write(&meuArquivo, csv_line, strlen(csv_line), &testeByte);
			res = f_close(&meuArquivo);
			f_mount(NULL, SDPath, 1);
		} else {
			printf("Falha ao montar Logical driver do Cartão sd \r\n");
		}
	}
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
	MX_SDMMC2_SD_Init();
	MX_FDCAN2_Init();
	MX_UART4_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
	HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	sd_write_sem = xSemaphoreCreateBinary();
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	can_data_queue = xQueueCreate(can_queue_len, sizeof(can_data_t));
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	xTaskCreate(Periodic_SD_save_task, "Periodic_SD_save_task", 4096, NULL, 8,
	NULL);
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_FDCAN_GetRxMessage(&hfdcan2,
		FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);

			printf("RX ID = 0x%03lX\r\n", RxHeader.Identifier);
		}

		timestamp_us = __HAL_TIM_GET_COUNTER(&htim3);

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
	if (HAL_SD_Init(&hsd2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SDMMC2_Init 2 */

	uint8_t sd_ok = 0;

	for (int tentativas = 0; tentativas < 10; tentativas++) {
		HAL_SD_DeInit(&hsd2);

		HAL_Delay(100);

		if (HAL_SD_Init(&hsd2) == HAL_OK) {
			sd_ok = 1;
			printf("SD inicializado na tentativa %d\r\n", tentativas + 1);
			break;
		}

		printf("Falha SD tentativa %d, erro=0x%08lx\r\n", tentativas + 1,
				hsd2.ErrorCode);

		HAL_Delay(500);
	}

	if (!sd_ok) {
		printf("Nao foi possivel inicializar SD\r\n");

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
	htim3.Init.Prescaler = 127;
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
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

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
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(sd_write_sem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
