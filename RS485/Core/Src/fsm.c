/*
 * fsm.c
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#include "fsm.h"
#include <stdio.h>
#include <stdbool.h>

#include "lcd.h"
#include "button.h"
#include "modbus_master.h"

/* Define --------------------------------------------------------------------*/
#define MAX_SLAVE_ID 2
#define AUTO_POLL_INTERVAL 2000 // 2 giây
#define EXECUTE_TIMEOUT 30000   // 30 giây

// !!!
#define BTN0_INDEX 0    // Nút Mode
#define BTN1_INDEX 1    // Nút Tăng / Đọc
#define BTN2_INDEX 2    // Nút Giảm / Ghi

/* Variables -----------------------------------------------------------------*/
extern UART_HandleTypeDef huart3;

// Biến FSM
static volatile FSM_State_t currentState = STATE_AUTO;
static volatile uint8_t target_slave_id = 1;

// Biến timer cho FSM (dùng HAL_GetTick)
static uint32_t auto_poll_timer = 0;
static uint32_t execute_mode_timer = 0;
static uint8_t auto_poll_target = 1;

// Buffer cho LCD
static char lcd_line[60];

/* Private Function Prototypes -----------------------------------------------*/
static void bytes_to_hex_string(uint8_t *bytes, uint16_t len, char *out_string);
static void clear_data_lines(void);
static void Update_LCD_Layout(FSM_State_t newState);

static void Handle_Auto_Poll(void);
static void UI_Handle_Read_Request(uint8_t slave_id);
static void UI_Handle_Write_Request(uint8_t slave_id);

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief  Khởi tạo FSM và các thành phần liên quan
 */
void FSM_Init(void) {
    lcd_init();     // Khởi tạo LCD
    button_init();  // Khởi tạo Button

    // Khởi tạo FSM và LCD layout ban đầu
    currentState = STATE_AUTO;
    Update_LCD_Layout(currentState);
}

/**
 * @brief  Hàm chạy chính của FSM (gọi liên tục trong while(1))
 */
void FSM_Run(void) {
    // Chỉ chạy Auto Poll khi ở trạng thái AUTO
    if (currentState == STATE_AUTO) {
        Handle_Auto_Poll();
    }
    // Chỉ kiểm tra Timeout khi ở trạng thái EXECUTE
    else if (currentState == STATE_EXECUTE) {
        if (HAL_GetTick() - execute_mode_timer > EXECUTE_TIMEOUT) {
            currentState = STATE_AUTO;
            Update_LCD_Layout(currentState);
        }
    }
}

/**
 * @brief  Xử lý logic nút nhấn (gọi mỗi 10ms từ ngắt timer)
 */
void FSM_Button_Handle(void) {
    // --- Xử lý Chuyển trạng thái (BTN0) ---
    // button_count[i] == 1 nghĩa là "vừa mới được nhấn" (chống lặp)
    if (button_count[BTN0_INDEX] == 1) {
        FSM_State_t nextState = currentState;
        if (currentState == STATE_AUTO) {
            nextState = STATE_SELECTED_ID;
        }
        else if (currentState == STATE_SELECTED_ID) {
            nextState = STATE_EXECUTE;
        }
        else if (currentState == STATE_EXECUTE) {
            nextState = STATE_AUTO;
        }

        if (nextState != currentState) {
            currentState = nextState;
            Update_LCD_Layout(currentState); // Cập nhật LCD
        }
    }

    // --- Xử lý các nút theo trạng thái ---
    switch (currentState)
    {
        case STATE_AUTO:
            // Không làm gì với BTN1, BTN2
            break;

        case STATE_SELECTED_ID:
            if (button_count[BTN1_INDEX] == 1) { // Tăng ID
                target_slave_id++;
                if (target_slave_id > MAX_SLAVE_ID) target_slave_id = 1;
                sprintf(lcd_line, "TARGET: S%d", target_slave_id);
                lcd_show_string(10, 50, lcd_line, BLUE, WHITE, 24, 0);
            }
            if (button_count[BTN2_INDEX] == 1) { // Giảm ID
                if (target_slave_id > 1) target_slave_id--;
                else target_slave_id = MAX_SLAVE_ID;
                sprintf(lcd_line, "TARGET: S%d", target_slave_id);
                lcd_show_string(10, 50, lcd_line, BLUE, WHITE, 24, 0);
            }
            break;

        case STATE_EXECUTE:
            if (button_count[BTN1_INDEX] == 1) { // Đọc (0x03)
                UI_Handle_Read_Request(target_slave_id);
                execute_mode_timer = HAL_GetTick(); // Reset timeout
            }
            if (button_count[BTN2_INDEX] == 1) { // Ghi (0x06)
                UI_Handle_Write_Request(target_slave_id);
                execute_mode_timer = HAL_GetTick(); // Reset timeout
            }
            break;
    }
}


/* Private Functions ---------------------------------------------------------*/

/**
 * @brief  Chuyển mảng byte sang chuỗi Hex để in
 */
static void bytes_to_hex_string(uint8_t *bytes, uint16_t len, char *out_string) {
    if (len == 0) { out_string[0] = '\0'; return; }
    for (int i = 0; i < len; i++) {
        sprintf(out_string + (i * 3), "%02X ", bytes[i]);
    }
    out_string[(len * 3) - 1] = '\0';
}

/**
 * @brief  Xóa các dòng dữ liệu trên LCD
 */
static void clear_data_lines(void) {
    const char *clear_str = "                          ";
    lcd_show_string(10, 50, clear_str, WHITE, WHITE, 24, 0);
    lcd_show_string(10, 80, clear_str, WHITE, WHITE, 24, 0);
    lcd_show_string(10, 66, clear_str, WHITE, WHITE, 16, 0);
    lcd_show_string(10, 106, clear_str, WHITE, WHITE, 16, 0);
}

/**
 * @brief  Cập nhật bố cục LCD khi chuyển trạng thái
 */
static void Update_LCD_Layout(FSM_State_t newState) {
    lcd_clear(WHITE);
    switch(newState)
    {
        case STATE_AUTO:
            sprintf(lcd_line, "Mode: AUTO");
            lcd_show_string(10, 10, lcd_line, BLACK, WHITE, 24, 0);
            lcd_show_string(10, 50, "S1 Temp:", BLACK, WHITE, 24, 0);
            lcd_show_string(10, 80, "S2 Temp:", BLACK, WHITE, 24, 0);
            auto_poll_target = 1;
            auto_poll_timer = HAL_GetTick();
            break;
        case STATE_SELECTED_ID:
            sprintf(lcd_line, "Mode: SELECT ID");
            lcd_show_string(10, 10, lcd_line, BLACK, WHITE, 24, 0);
            sprintf(lcd_line, "TARGET: S%d", target_slave_id);
            lcd_show_string(10, 50, lcd_line, BLUE, WHITE, 24, 0);
            break;
        case STATE_EXECUTE:
            sprintf(lcd_line, "Mode: EXECUTE (S%d)", target_slave_id);
            lcd_show_string(10, 10, lcd_line, BLACK, WHITE, 24, 0);
            lcd_show_string(10, 50, "Sent:", BLACK, WHITE, 16, 0);
            lcd_show_string(10, 90, "Recv:", BLACK, WHITE, 16, 0);
            execute_mode_timer = HAL_GetTick();
            break;
    }
}

/**
 * @brief  Xử lý logic chế độ AUTO (poll và cập nhật)
 */
static void Handle_Auto_Poll(void) {
    if (HAL_GetTick() - auto_poll_timer < AUTO_POLL_INTERVAL) return;

    int16_t read_data[1];
    uint8_t raw_data[7];
    uint16_t y_pos = (auto_poll_target == 1) ? 50 : 80;

    lcd_show_string(150, y_pos, "       ", WHITE, WHITE, 24, 0);

    if (Modbus_Read_Registers_Raw(&huart3, auto_poll_target, 0, 1, read_data, raw_data) == HAL_OK) {
        sprintf(lcd_line, "%d", read_data[0]);
        lcd_show_string(150, y_pos, lcd_line, GREEN, WHITE, 24, 0);
    } else {
        lcd_show_string(150, y_pos, "ERROR", RED, WHITE, 24, 0);
    }

    auto_poll_target = (auto_poll_target == 1) ? 2 : 1;
    auto_poll_timer = HAL_GetTick();
}

/**
 * @brief  Gửi Lệnh Đọc (0x03) và cập nhật UI
 */
static void UI_Handle_Read_Request(uint8_t slave_id) {
    clear_data_lines();
    uint8_t txData[8];
    uint16_t reg_addr = 0, num_regs = 1;

    txData[0] = slave_id; txData[1] = 0x03;
    txData[2] = (reg_addr >> 8); txData[3] = reg_addr & 0xFF;
    txData[4] = (num_regs >> 8); txData[5] = num_regs & 0xFF;
    uint16_t crc = Modbus_CRC16(txData, 6);
    txData[6] = crc & 0xFF; txData[7] = (crc >> 8) & 0xFF;

    bytes_to_hex_string(txData, 8, lcd_line);
    lcd_show_string(10, 66, lcd_line, BLUE, WHITE, 16, 0);

    int16_t parsed_data[1];
    uint8_t rx_buffer_size = 5 + (num_regs * 2);
    uint8_t raw_rx_data[rx_buffer_size];

    if (Modbus_Read_Registers_Raw(&huart3, slave_id, reg_addr, num_regs, parsed_data, raw_rx_data) == HAL_OK) {
        bytes_to_hex_string(raw_rx_data, rx_buffer_size, lcd_line);
        lcd_show_string(10, 106, lcd_line, GREEN, WHITE, 16, 0);
    } else {
        lcd_show_string(10, 106, "ERROR (Timeout/CRC)", RED, WHITE, 16, 0);
    }
}

/**
 * @brief  Gửi Lệnh Ghi (0x06) - Ghi 0 - và cập nhật UI
 */
static void UI_Handle_Write_Request(uint8_t slave_id) {
    clear_data_lines();
    uint8_t txData[8];
    uint16_t reg_addr = 0; int16_t value = 0;

    txData[0] = slave_id; txData[1] = 0x06;
    txData[2] = (reg_addr >> 8); txData[3] = reg_addr & 0xFF;
    txData[4] = (value >> 8);    txData[5] = value & 0xFF;
    uint16_t crc = Modbus_CRC16(txData, 6);
    txData[6] = crc & 0xFF; txData[7] = (crc >> 8) & 0xFF;

    bytes_to_hex_string(txData, 8, lcd_line);
    lcd_show_string(10, 66, lcd_line, BLUE, WHITE, 16, 0);

    uint8_t raw_rx_data[8];
    if (Modbus_Write_Register_Raw(&huart3, slave_id, reg_addr, value, raw_rx_data) == HAL_OK) {
        bytes_to_hex_string(raw_rx_data, 8, lcd_line);
        lcd_show_string(10, 106, lcd_line, GREEN, WHITE, 16, 0);
    } else {
        lcd_show_string(10, 106, "ERROR (Timeout/CRC)", RED, WHITE, 16, 0);
    }
}
