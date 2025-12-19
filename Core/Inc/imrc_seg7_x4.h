#ifndef IMRC_SEG7_X4_H
#define IMRC_SEG7_X4_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>

void seg7_init(uint8_t _d_pin[4], GPIO_TypeDef *_d_port[4],
               uint8_t _ser_pin, GPIO_TypeDef *_ser_port,
               uint8_t _rclk_pin, GPIO_TypeDef *_rclk_port,
               uint8_t _srclk_pin, GPIO_TypeDef *_srclk_port);
uint8_t seg7_char_to_byte(char c);
void seg7_drive(uint8_t digit, uint8_t c);
void seg7_print(char str[], uint8_t len, bool isPaddingRight, bool isPeriodIndependent);

#endif