/*
 * fsm_slave.c
 *
 *  Created on: Jan 15, 2026
 *      Author: DELL
 */


#include "fsm_slave.h"
#include "lcd.h"
#include "button.h"
#include <stdio.h>

/* --- CẤU HÌNH FLASH (STM32F407) --- */
// Sector 11 là sector cuối cùng của 1MB Flash
#define FLASH_USER_START_ADDR   ((uint32_t)0x080E0000)

/* --- ĐỊNH NGHĨA NÚT NHẤN (Theo phần cứng của bạn) --- */
#define BTN_MODE    2  // Nút 0: Vào Setup / Lưu & Thoát
#define BTN_UP      3  // Nút 1: Tăng
#define BTN_DOWN    7  // Nút 2: Giảm

/* --- BIẾN TOÀN CỤC --- */
uint8_t my_slave_id = 1;

/* --- BIẾN NỘI BỘ (STATIC) --- */
static Slave_State_t currentState = SLAVE_STATE_IDLE;
static uint8_t temp_id = 1;           // ID tạm thời khi đang chỉnh
static uint32_t saving_timer = 0;     // Timer đếm thời gian hiển thị thông báo
static uint8_t lcd_needs_update = 1;  // Cờ báo cần vẽ lại màn hình (tối ưu hiệu năng)

/* --- CÁC HÀM XỬ LÝ FLASH (PRIVATE) --- */
static void Save_Slave_ID_To_Flash(uint16_t id) {
    HAL_FLASH_Unlock();
    // Xóa cờ lỗi cũ
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    // Xóa Sector 11
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_11;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
        // Ghi ID mới (Half-word)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR, id);
    }
    HAL_FLASH_Lock();
}

static uint16_t Load_Slave_ID_From_Flash(void) {
    uint16_t id = *(__IO uint16_t*)FLASH_USER_START_ADDR;
    if (id == 0xFFFF || id == 0x0000 || id > 247) return 1; // Giá trị mặc định
    return id;
}

/* --- CÁC HÀM PUBLIC --- */

/**
 * @brief Khởi tạo FSM, LCD, Button và Load ID
 */
void FSM_Slave_Init(void) {
    lcd_init();
    button_init();

    // 1. Load ID từ Flash
    my_slave_id = (uint8_t)Load_Slave_ID_From_Flash();
    temp_id = my_slave_id;

    // 2. Kiểm tra nút MODE ngay khi khởi động
    // Nếu giữ nút MODE khi cấp nguồn -> Vào thẳng Setup
    button_scan();
    if (button_count[BTN_MODE] > 0) {
        currentState = SLAVE_STATE_SETUP;
    } else {
        currentState = SLAVE_STATE_IDLE;
    }

    lcd_needs_update = 1; // Yêu cầu vẽ màn hình lần đầu
}

/**
 * @brief Logic chính, gọi trong while(1)
 */
void FSM_Slave_Run(void) {
    char buf[30];

    switch (currentState) {
        case SLAVE_STATE_IDLE:
            // Chỉ vẽ lại khi cần thiết để tránh nháy màn hình
            if (lcd_needs_update) {
                lcd_clear(BLACK);
                sprintf(buf, "SLAVE ID: %02d", my_slave_id);
                lcd_show_string(10, 10, buf, WHITE, BLACK, 24, 0);
                lcd_show_string(10, 50, "Modbus: Ready", GREEN, BLACK, 16, 0);

                // Hướng dẫn vào setup
                lcd_show_string(10, 220, "Hold BTN0 to Setup", GRAY, BLACK, 12, 0);

                lcd_needs_update = 0;
            }
            break;

        case SLAVE_STATE_SETUP:
            if (lcd_needs_update) {
                lcd_clear(BLUE);
                lcd_show_string(30, 20, "SETUP ID MODE", WHITE, BLUE, 24, 0);

                // Hiển thị số ID to ở giữa
                sprintf(buf, "%02d", temp_id);
                lcd_show_string(130, 100, buf, YELLOW, BLUE, 32, 0);

                // Hướng dẫn
                lcd_show_string(10, 200, "BTN1: (+) | BTN2: (-)", WHITE, BLUE, 16, 0);
                lcd_show_string(10, 220, "BTN0: SAVE & EXIT", WHITE, BLUE, 16, 0);

                lcd_needs_update = 0;
            }
            break;

        case SLAVE_STATE_SAVING:
            if (lcd_needs_update) {
                lcd_clear(BLACK);
                lcd_show_string(80, 110, "SAVING...", GREEN, BLACK, 24, 0);
                lcd_needs_update = 0;

                // Lưu ID vào Flash ngay lập tức
                Save_Slave_ID_To_Flash(temp_id);
                my_slave_id = temp_id;

                // Bắt đầu đếm giờ để hiện chữ Saving trong 1 giây
                saving_timer = HAL_GetTick();
            }

            // Kiểm tra timeout (Non-blocking delay)
            if (HAL_GetTick() - saving_timer > 1000) {
                currentState = SLAVE_STATE_IDLE; // Chuyển về màn hình chính
                lcd_needs_update = 1;
            }
            break;
    }
}

/**
 * @brief Xử lý nút nhấn, gọi trong ngắt Timer 10ms
 */
void FSM_Slave_Button_Handle(void) {
    // 1. Xử lý khi ở chế độ SETUP
    if (currentState == SLAVE_STATE_SETUP) {
        // Tăng
        if (button_count[BTN_UP] == 1) {
            temp_id++;
            if (temp_id > 20) temp_id = 1; // Giới hạn max ID
            lcd_needs_update = 1;
        }
        // Giảm
        if (button_count[BTN_DOWN] == 1) {
            if (temp_id > 1) temp_id--;
            else temp_id = 20;
            lcd_needs_update = 1;
        }
        // Lưu (Nhấn nhả BTN_MODE)
        if (button_count[BTN_MODE] == 1) {
            currentState = SLAVE_STATE_SAVING;
            lcd_needs_update = 1;
        }
    }

    // 2. Xử lý khi ở chế độ IDLE (Để vào Setup mà không cần Reset mạch)
    else if (currentState == SLAVE_STATE_IDLE) {
        // Nếu giữ nút MODE quá 3 giây (300 * 10ms)
        if (button_count[BTN_MODE] > 300) {
            temp_id = my_slave_id; // Reset temp_id về ID hiện tại
            currentState = SLAVE_STATE_SETUP;
            lcd_needs_update = 1;

            // Reset bộ đếm nút để tránh lặp lại sự kiện
            //button_count[BTN_MODE] = 0;
        }
    }
}
