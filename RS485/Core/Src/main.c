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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm_slave.h"
#include "button.h"
#include "modbus_crc.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// SLAVE 1: Đặt giá trị này là 1
// SLAVE 2: Đặt giá trị này là 2
#define MY_SLAVE_ID 1
//
#define MAX_REGISTERS 10    // Số lượng thanh ghi "giả"
#define RX_BUFFER_SIZE 256  // Kích thước buffer nhận tối đa

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

//// Đây là "dữ liệu giả" (holding registers)
int16_t holding_registers[MAX_REGISTERS] = {0};

//// Buffer nhận dữ liệu qua UART
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_len = 0; // Kích thước dữ liệu ngắt nhận được

// Biến my_slave_id đã được extern trong fsm_slave.h, không cần khai báo lại
extern uint8_t my_slave_id;

// Dữ liệu giả lập
//int16_t holding_registers[10] = {0};
uint8_t rx_buffer[256];
volatile uint8_t timer_10ms_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FSMC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// Khai báo các hàm helper cho Slave
void RS485_TX_Enable(void);
void RS485_RX_Enable(void);
void Modbus_Send_Response(uint8_t *data, uint16_t len);
void Handle_Read_Request(uint8_t *frame);
void Handle_Write_Request(uint8_t *frame);
void Process_Modbus_Frame(uint8_t *frame, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Kích hoạt chế độ GỬI (TX) trên RS485.
 */
void RS485_TX_Enable(void) {
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Kích hoạt chế độ NHẬN (RX) trên RS485.
 */
void RS485_RX_Enable(void) {
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Gửi gói tin phản hồi
 */
void Modbus_Send_Response(uint8_t *data, uint16_t len) {
    RS485_TX_Enable();
    HAL_UART_Transmit(&huart3, data, len, 100);
    RS485_RX_Enable(); // Chuyển về nhận ngay
}

/**
 * @brief  Xử lý yêu cầu Đọc (0x03)
 */
void Handle_Read_Request(uint8_t *frame) {
    uint16_t reg_addr = (frame[2] << 8) | frame[3];
    uint16_t num_regs = (frame[4] << 8) | frame[5];

    // Kiểm tra địa chỉ có hợp lệ không
    if ((reg_addr + num_regs) > MAX_REGISTERS) {
        return;
    }

    // Gói tin phản hồi: 1(ID) + 1(FC) + 1(Byte Count) + N*2(Data) + 2(CRC)
    uint8_t resp_len = 5 + (num_regs * 2);
    uint8_t resp[resp_len];

    resp[0] = my_slave_id;
    resp[1] = 0x03;
    resp[2] = num_regs * 2; // Số byte dữ liệu

    // Lấy data từ "dữ liệu giả" (Gửi High Byte trước)
    for (int i = 0; i < num_regs; i++) {
        resp[3 + i*2] = (holding_registers[reg_addr + i] >> 8) & 0xFF;
        resp[4 + i*2] = holding_registers[reg_addr + i] & 0xFF;
    }

    uint16_t crc = Modbus_CRC16(resp, resp_len - 2);
    resp[resp_len - 2] = crc & 0xFF;
    resp[resp_len - 1] = (crc >> 8) & 0xFF;

    Modbus_Send_Response(resp, resp_len);
}

/**
 * @brief  Xử lý yêu cầu Ghi (0x06)
 */
void Handle_Write_Request(uint8_t *frame) {
    uint16_t reg_addr = (frame[2] << 8) | frame[3];
    int16_t value = (frame[4] << 8) | frame[5];

    if (reg_addr >= MAX_REGISTERS) {
        return; // (Gửi mã lỗi)
    }

    // Ghi giá trị vào "dữ liệu giả"
    holding_registers[reg_addr] = value;

    // Phản hồi của 0x06 là một bản sao (echo)
    Modbus_Send_Response(frame, 8);
}

/**
 * @brief  Xử lý gói tin Modbus (được gọi từ ngắt)
 */
void Process_Modbus_Frame(uint8_t *frame, uint16_t len) {
    // 1. Kiểm tra ID
    if (frame[0] != my_slave_id) {
        return; // Không phải cho mình, bỏ qua
    }

    // 2. Kiểm tra CRC
    if (Modbus_Validate_CRC(frame, len) != 1) {
        return; // Lỗi CRC, bỏ qua
    }

    // 3. Phân tích Function Code
    switch(frame[1]) {
        case 0x03: // Read Holding Registers
            Handle_Read_Request(frame);
            break;
        case 0x06: // Write Single Register
            Handle_Write_Request(frame);
            break;
        default:
            // (Không hỗ trợ mã hàm này, có thể gửi mã lỗi)
            break;
    }
}

// --- CHANGED: Thêm Callback Timer để bật cờ 10ms ---
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        timer_10ms_flag = 1;
    }
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
  MX_USART3_UART_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 1. Khởi tạo FSM (LCD, Button, Flash, Load ID)
  FSM_Slave_Init();

  // 2. Bắt đầu Timer ngắt
    HAL_TIM_Base_Start_IT(&htim2);

    // 3. Modbus Config
    holding_registers[0] = 250; // 25.0 độ C
    RS485_RX_Enable();
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, 256); // Sửa lại size buffer số cứng nếu cần
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // 1. Chạy FSM liên tục (Cập nhật màn hình, chuyển trạng thái)
        FSM_Slave_Run();

        // 2. Tác vụ định kỳ 10ms (Thay thế HAL_Delay)
        if (timer_10ms_flag == 1) {
            timer_10ms_flag = 0; // Xóa cờ

            // Quét nút và xử lý logic nút cho FSM
            button_scan();
            FSM_Slave_Button_Handle();

            // --- LOGIC MÔ PHỎNG NHIỆT ĐỘ (Non-blocking) ---
            static uint8_t sim_tick = 0;
            sim_tick++;
            if (sim_tick >= 100) { // 10ms * 100 = 1000ms (1 giây)
                sim_tick = 0;

                // Tăng nhiệt độ giả
                if (holding_registers[0] != 0) {
                    holding_registers[0]++;
                }
                if (holding_registers[0] > 500) {
                    holding_registers[0] = 250;
                }
            }
        }
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 839;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSMC_RES_GPIO_Port, FSMC_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_LATCH_GPIO_Port, LD_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FSMC_BLK_Pin|RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FSMC_RES_Pin */
  GPIO_InitStruct.Pin = FSMC_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSMC_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_LATCH_Pin */
  GPIO_InitStruct.Pin = LD_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_LATCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FSMC_BLK_Pin RS485_EN_Pin */
  GPIO_InitStruct.Pin = FSMC_BLK_Pin|RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_LOAD_Pin */
  GPIO_InitStruct.Pin = BTN_LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_LOAD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 0xf;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 60;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 8;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 9;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Hàm Callback ngắt UART Rx Event (Khi nhận xong 1 gói tin)
  * @note   Hàm này được HAL gọi tự động khi ngắt UART xảy ra.
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. Kiểm tra xem có đúng là ngắt từ UART3 không
    if (huart->Instance == USART3) {

        // 2. Ghi lại độ dài gói tin đã nhận
        rx_len = Size;

        // 3. Gọi hàm xử lý gói tin Modbus
        Process_Modbus_Frame(rx_buffer, rx_len);

        // 4. Kích hoạt lại ngắt nhận cho gói tin tiếp theo
        //    Đây là bước bắt buộc!
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, RX_BUFFER_SIZE);
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
