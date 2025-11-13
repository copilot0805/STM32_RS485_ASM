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
#include "modbus_crc.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// SLAVE 1: Đặt giá trị này là 1
// SLAVE 2: Đặt giá trị này là 2
#define MY_SLAVE_ID 1

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
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Đây là "dữ liệu giả" (holding registers)
int16_t holding_registers[MAX_REGISTERS] = {0};

// Buffer nhận dữ liệu qua UART
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_len = 0; // Kích thước dữ liệu ngắt nhận được

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
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

    resp[0] = MY_SLAVE_ID;
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
    if (frame[0] != MY_SLAVE_ID) {
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
  /* USER CODE BEGIN 2 */

  // Mô phỏng dữ liệu (ví dụ: nhiệt độ ở thanh ghi 0)
    holding_registers[0] = 253; // 25.3 độ C

    RS485_RX_Enable(); // Slave luôn ở chế độ Nhận

    // Kích hoạt ngắt UART Receive to Idle (Tốt nhất cho Modbus)
    // Nó sẽ kích hoạt ngắt khi đường truyền rảnh (idle)
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Làm cho "dữ liệu giả" sống động
	      HAL_Delay(1000);

	      // Chỉ tăng nếu nó không bị Master reset về 0
	      if (holding_registers[0] != 0) {
	          holding_registers[0]++;
	      }

	      // Giới hạn nhiệt độ (ví dụ)
	      if (holding_registers[0] > 500) {
	          holding_registers[0] = 250;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
