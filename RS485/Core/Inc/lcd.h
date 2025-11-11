/*
 * lcd.h
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

/* Định nghĩa các màu cơ bản (RGB 5-6-5) */
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define BRED        0XF81F
#define GRED        0XFFEO
#define GBLUE       0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN       0XBC40
#define BRRED       0XFC07
#define GRAY        0X8430
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458
#define LIGHTGREEN  0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY       0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

/* Cấu trúc LCD */
typedef struct {
	uint16_t width;   // Chiều rộng
	uint16_t height;  // Chiều cao
	uint16_t id;      // ID của chip (ví dụ: 0x9341)
} _lcd_dev;

// Biến toàn cục chứa thông số LCD
extern _lcd_dev lcddev;

// Định nghĩa hướng xoay màn hình
#define DFT_SCAN_DIR 0

/* Khai báo các hàm public */
void lcd_init(void);
void lcd_clear(uint16_t color);
void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_circle(int xc, int yc, uint16_t c, int r, int fill);
void lcd_show_char(uint16_t x, uint16_t y, uint8_t character, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void lcd_show_string(uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void lcd_show_string_center(uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void lcd_show_int_num(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
void lcd_show_float_num(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
void lcd_show_picture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);
void lcd_set_direction(uint8_t dir);
void lcd_set_cursor(uint16_t x, uint16_t y);
void lcd_set_display_on(void);
void lcd_set_display_off(void);

#endif /* INC_LCD_H_ */
