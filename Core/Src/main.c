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
//#include "isotp_functions.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
CAN_FilterTypeDef   canfil;
CAN_RxHeaderTypeDef rxHeader;
uint8_t             canRX[8];
CAN_TxHeaderTypeDef txHeader;
uint8_t             canTX[8];
uint32_t            txMailbox;

/*
 * TXRXState codes/definitions:
 * 0 - not sending/receiving anything
 * 1 - transmitter state, sent FF, awaiting FC
 * 2 - transmitter state, sending CFs
 * 3 - receiver state
 * 4 - message received, print on next main iteration then TXRXState=0
 */
uint8_t             TXRXState;
uint8_t             myTXBuffer[128];
uint16_t			myTXBufferLength;
uint16_t            myTXBufferIndex;
uint16_t			myTXDataLength;
uint8_t             myRXBuffer[128];
uint16_t			myRXBufferLength;
uint16_t            myRXBufferIndex;
uint16_t			myRXDataLength;
uint16_t			myCFsExpected; // this is used as a collection of boolean bits
uint16_t			myCFsReceived; // this is used as a collection of boolean bits

/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void manageTXRXState(void);						//--+
uint8_t CAN_Send(uint8_t *data, uint8_t len);		//	|
void CAN_SendSFFF(uint8_t *data, uint16_t length);//	+-- These are handled by the import now.
void CAN_SendCFs();								//	|
void setCFsExpected(void);						//	|
void routeRX(void);								//--+
void CAN_SendArray(uint8_t *data, uint16_t length);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

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
	CAN_RxHeaderTypeDef rxHeader;    // CAN receive header
	CAN_TxHeaderTypeDef txHeader;    // CAN transmit header
	uint8_t csend[8] = {1,2,3,4,5,6,7,8}; // CAN TX buffer
	CAN_FilterTypeDef canfil;        // CAN filter
	uint32_t canMailbox;

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
	MX_LPUART1_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_RTC_Init();
	MX_IWDG_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */
	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil.FilterIdHigh = 0;
	canfil.FilterIdLow = 0;
	canfil.FilterMaskIdHigh = 0;
	canfil.FilterMaskIdLow = 0;
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1,&canfil);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	/* USER CODE BEGIN CAN_TX_INIT */
	txHeader.StdId = 0x123;              // CAN ID you want
	txHeader.ExtId = 0;                  // unused for standard ID
	txHeader.IDE   = CAN_ID_STD;         // standard frame
	txHeader.RTR   = CAN_RTR_DATA;       // data frame
	txHeader.DLC   = 8;                  // 0–8 bytes
	txHeader.TransmitGlobalTime = DISABLE;
	/* USER CODE END CAN_TX_INIT */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Initialize buffer details
	myTXBufferLength = 128; // 128 chosen during initialization above
	myRXBufferLength = 128; // 128 chosen during initialization above
	myTXBufferIndex = 0;
	myRXBufferIndex = 0;
	myTXDataLength = 0;
	myRXDataLength = 0;
	myCFsExpected = 0;
	myCFsReceived = 0;
	// Setup state
	txHeader.StdId = 0x210;
	TXRXState = 0;
	uint8_t retries = 0;
	while (1)
	{
		/* USER CODE END WHILE */
		static uint32_t lastTick = 0;
		if (HAL_GetTick() - lastTick > 250) // every 250 ms
		{
			lastTick = HAL_GetTick();
			manageTXRXState();
		}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  //hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  //hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  //hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  //hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 1000;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
  RTC_AlarmTypeDef sAlarm;
  HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);
  if(sAlarm.AlarmTime.Seconds>58) {
    sAlarm.AlarmTime.Seconds=0;
  }else{
    sAlarm.AlarmTime.Seconds=sAlarm.AlarmTime.Seconds+1;
  }
//    while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3); // RX activity LED
		printf("CAN RX | ID: 0x%03lX | DLC: %d | DATA:",
			   rxHeader.StdId,
			   rxHeader.DLC);

		for (uint8_t i = 0; i < rxHeader.DLC; i++)
		{
			printf(" %02X", canRX[i]);
		}

		printf("\r\n");

		routeRX();
	}
}


/*
 * Determines what should be done from the synchronous code side.
 * This should be run regularly (in the 1-10 Hz range) in main to manage the TX/RX states
 */
void manageTXRXState(void)
{
	// Send data if not sending
	if (TXRXState == 0)
	{
		// Build test data
		myTXDataLength = 32;
		for(int i=0; i<32; i++)
		{
			myTXBuffer[i]=i;
		}
		// Send test data
		//CAN_SendSFFF(myTXBuffer, myTXDataLength);
	}
	else if (TXRXState == 1)
	{
		retries++;
		if (retries >= 10)
		{
			TXRXState = 0; // Allow PQ to retry send on next iteration
		}
	}
	else if (TXRXState == 2)
	{
		// do nothing
	}
	else if (TXRXState == 3)
	{
		// do nothing
	}
	else if (TXRXState == 4)
	{
		printf("RX Buffer:");
		for (uint16_t i = 0; i < myRXDataLength; i++)
		{
			printf(" i=%02d %02X,", i, myRXBuffer[i]);
		}
		printf("\n");
		TXRXState = 0;
	}
}


/*
 * Safely sends (and prints to terminal) one CAN frame.
 */
uint8_t CAN_Send(uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;
    txHeader.DLC = len;
    printf("checking CAN TX mailboxes.\r\n");
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox) == HAL_OK)
        {
            printf("CAN TX | ID: 0x%03lX | DLC: %d\r\n",
                   txHeader.StdId, txHeader.DLC);
        }
        else
        {
            printf("CAN TX ERROR\r\n");
        }
        return 1;
    }
    printf("CAN TX mailboxes full, failed to send data.\r\n");
    return 0;
}


/*
 * The entry-point for sending data via CAN ISO-TP.
 * Decides automatically whether to send a SF or FF.
 */
void CAN_SendSFFF(uint8_t *data, uint16_t length)
{
	uint8_t packet[8] = {0};
	myTXBufferIndex = 0;

	// Single Frame (SF): up to 7 bytes payload
	if (length <= 7)
	{
	    packet[0] = (0x0u << 4) | (uint8_t)(length & 0x0Fu);
	    memcpy(&packet[1], data, length);
	    CAN_Send(packet, 8);   // send full 8 bytes for consistent logging
	    TXRXState = 0; // Code for not actively sending out data
	    return;
	}
	// (End SF)

	// First Frame (FF): 12 bit length -> 4095 byte data limit
	if (length>4095){
		// ERROR: Above the first frame memory limit, return an handle error
		TXRXState = 0; // Code for not actively sending out data
		return;

	}

	// PCI: First Frame (1) + 12-bit length
	packet[0] = (0x1u << 4) | (uint8_t)((length >> 8) & 0x0Fu);
    packet[1] = (uint8_t)(length & 0xFFu);

    // First 6 data bytes go in bytes 2..7
	memcpy(&packet[2], &data[0], 6);
	myTXBufferIndex += 6;

	CAN_Send(packet, 8);
	TXRXState = 1; // Code for having sent FF
	// IMPORTANT: Will send CFs after FC received from receiver board
}


/*
 * Sends the next batch of CFs from the data.
 * Note: ensure that the TXBuffer has not been modified since the FF or previous CF was sent.
 */
void CAN_SendCFs()
{
	if ((canRX[0] & 0xF0) != 0x30) return; // Did not receive FC frame. CAN_SendCF was called incorrectly
	if (TXRXState != 1 && TXRXState != 2) return; // Doesn't have CFs to send.
	// Determine 0=continue, 1=wait, 2=abort
	uint8_t pcl = canRX[0] & 0x0F;
	if (pcl == 1)
	{
		return;
	}
	else if (pcl != 0)
	{
		TXRXState = 0; // Abort data send process
		return;
	}
	// If here, should continue sending bytes!
	TXRXState = 2; // Code for actively sending CFs
	// Parse send parameters
	uint8_t numFrames = canRX[1];
	uint8_t frameWait = canRX[2]; // Note this is technically incorrect ISO-TP, but okay for 10ms standard delay

	uint8_t sn = 1;

	for (uint8_t i = 0; i <= numFrames; i++) //  changed loop so data is sent when numframes is not equal to 0
	{
		printf("sending frame %d\r\n", i);
		uint8_t packet[8] = {0};

		// PCI: 2 then index
		packet[0] = (0x2u << 4) | sn;
		sn = (sn + 1) & 0x0F;
		if (sn == 0) sn = 1;

		// Prep next data bytes
		uint16_t numRemaining = myTXDataLength - myTXBufferIndex;
		uint8_t numSend = numRemaining > 7 ? 7 : numRemaining;
		memcpy(&packet[1], &myTXBuffer[myTXBufferIndex], numSend);
		myTXBufferIndex += numSend;

		CAN_Send(packet, 8);

		//HAL_Delay(frameWait); Our FCs will have no frame wait delay - they can arrive out of order!

		if (myTXBufferIndex >= myTXDataLength)
		{
			TXRXState = 0; // Done sending data!
			break;
			// Note: this doesn't reset the dataIndex pointer.
		}
	}
}


/*
 * Helper function that determines the expected shape of the next incoming CF bundle.
 * Also determines (and stores in LSB) if at least 1 more CF bundle is required.
 */
void setCFsExpected(void)
{
	myCFsExpected = 0;
	uint16_t bytes_expected = myRXDataLength - myRXBufferIndex;
	uint16_t numCFs = (bytes_expected + 6) / 7; // Number of CFs needed (ceil division)
	uint8_t numCFs_this_block = (numCFs > 15) ? 15 : numCFs; // Limit to current block (max 15)
	for (uint8_t k = 1; k <= numCFs_this_block; k++) // Set bits 1–numCFs_this_block
	{
	    myCFsExpected |= (uint16_t)(1u << k);
	}
	if (numCFs > 15) // If more CFs are needed beyond this block, set LSB
	{
	    myCFsExpected |= 0x1; // now should = 0xFFFF
	}
}


/*
 * Takes action based on the RX frame type.
 */
void routeRX(void)
{
	// Get Frame Type
	uint8_t frameType = canRX[0] >> 4;

	if (frameType == 0)
	{
		// Case incoming frame is SF
		myRXDataLength = canRX[0] & 0x0F;
		memcpy(myRXBuffer, &canRX[1], myRXDataLength);
		myRXBufferIndex = myRXDataLength;
		// The data portion of the SF should be in myRXBuffer
		// Consider validating that the SF has been digested properly here
	}
	else if (frameType == 1)
	{
		// Case incoming frame is FF
		TXRXState = 3;
		myRXDataLength = 0x0FFF & ((((uint16_t)canRX[0]) << 8) | (uint16_t)canRX[1]);
		memcpy(myRXBuffer, &canRX[2], 6);
		myRXBufferIndex = 6;
		// The data portion of the FF should be in myRXBuffer (no action)
		// Consider validating that the FF has been digested properly here
		// Decide how much data to expect from the next CF dump
		setCFsExpected();
		// Send FC Frame
		myCFsReceived = 0;
		uint8_t packet[8] = {0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // requests 15 CFs
		CAN_Send(packet, 8);
	}
	else if (frameType == 2)
	{
		// Case incoming frame is CF
		if (TXRXState != 3)
		{
			return; // Should not be receiving CFs at the moment
		}
		// ID the CF
		uint8_t CFsn = canRX[0] & 0x0F;
		// Store in buffer in correct location
		uint16_t write_index = myRXBufferIndex + 7u * (CFsn - 1);
		if (write_index >= myRXDataLength)
		{
			return;
		}
		uint16_t bytes_remaining = myRXDataLength - write_index;
		uint8_t bytes_to_copy = (bytes_remaining >= 7) ? 7 : (uint8_t)bytes_remaining;
		memcpy(&myRXBuffer[write_index], &canRX[1], bytes_to_copy);
		// Note received
		myCFsReceived |= (uint16_t)(1u << CFsn);
		// Check for message complete
		if (myCFsReceived == myCFsExpected)
		{
			TXRXState = 4;
		}
		// if received all CFs, send FC Frame
		else if (myCFsReceived == 0xFFFE && (myCFsExpected & 0x1)) // All received when highest 15 bits are 1
		{
			myRXBufferIndex += 105; // 105 = 15*7
			// Decide how much data to expect from the next CF dump
			setCFsExpected();
			// Send CF
			myCFsReceived = 0;
			uint8_t packet[8] = {0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // requests 15 CFs
			CAN_Send(packet, 8);
		}
	}
	else if (frameType == 3)
	{
		// Case incoming message is FC
		// Flag the FC
		uint8_t FCflag = canRX[0] & 0x0F;
		// Case by case
		if (FCflag == 0)
		{
			// Case continue with CFs
			CAN_SendCFs();
		}
		else if (FCflag == 1)
		{
			// Case wait
			// do nothing. The state machine will use this for retry logic later
		}
		else if (FCflag == 2)
		{
			// Case abort
			TXRXState = 0;
		}
	}
}


/*
 * Old CAN Testing function incompatible with ISO-TP.
 * Do not use.
 */
void CAN_SendArray(uint8_t *data, uint16_t length)
{
    uint16_t index = 0;
    uint8_t packet[8];

    while (index < length)
    {
        uint8_t chunk = (length - index > 8) ? 8 : length - index;

        memcpy(packet, &data[index], chunk);

        CAN_Send(packet, chunk);

        index += chunk;

        HAL_Delay(2);
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
