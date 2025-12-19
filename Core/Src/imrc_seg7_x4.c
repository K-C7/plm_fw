#include "imrc_seg7_x4.h"
#include "stm32f303x8.h"
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>

#define SEG7_DISPLAY_DELAY 1

const uint8_t seg_char_num[10] = {
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100100, // 7
    0b11111110, // 8
    0b11110110, // 9
};

const uint8_t seg_char_sign[] = {
    0b00000000, // space
    0b00000001, // .
    0b11111111, // seg_test
    0b00000010, // -
    0b00010000, // _
    0b00010010, // =
    0b11000110, // °
    0b01000100, // "
};

const uint8_t seg_char_alp[26] = {
    // A, B(b), C, D(d), E, F, G, H
    0b11101110, // A : 0xEE
    0b00111110, // b : 0x3E (小文字)
    0b10011100, // C : 0x9C
    0b01111010, // d : 0x7A (小文字)
    0b10011110, // E : 0x9E
    0b10001110, // F : 0x8E
    0b10111100, // G : 0xBC (6のa無し版)
    0b01101110, // H : 0x6E

    // I, J, K, L, M, N, O, P
    0b01100000, // I : 0x60 (1と同じ)
    0b01111000, // J : 0x78
    0b01101110, // K : 0x6E (Hと同じ、または表記不可)
    0b00011100, // L : 0x1C
    0b00101010, // M : 0x2A (nと同じ、または表記不可)
    0b00101010, // n : 0x2A (小文字)
    0b11111100, // O : 0xFC (0と同じ)
    0b11001110, // P : 0xCE

    // Q, R, S, T, U, V, W, X
    0b11100110, // q : 0xE6
    0b00001010, // r : 0x0A (小文字)
    0b10110110, // S : 0xB6 (5と同じ)
    0b00011110, // t : 0x1E (小文字)
    0b01111100, // U : 0x7C
    0b01111100, // V : 0x7C (Uと同じ)
    0b01111100, // W : 0x7C (Uと同じ、または表記不可)
    0b01101110, // X : 0x6E (Hと同じ)

    // Y, Z
    0b01110110, // y : 0x76
    0b11011010  // Z : 0xDA (2と同じ)
};

uint8_t d_pin[4];
GPIO_TypeDef *d_port[4];

uint8_t ser_pin;
GPIO_TypeDef *ser_port;
uint8_t rclk_pin;
GPIO_TypeDef *rclk_port;
uint8_t srclk_pin;
GPIO_TypeDef *srclk_port;

void seg7_init(uint8_t _d_pin[4], GPIO_TypeDef *_d_port[4],
               uint8_t _ser_pin, GPIO_TypeDef *_ser_port,
               uint8_t _rclk_pin, GPIO_TypeDef *_rclk_port,
               uint8_t _srclk_pin, GPIO_TypeDef *_srclk_port)
{
    for (int i = 0; i < 4; i++)
    {
        d_pin[i] = _d_pin[i];
        d_port[i] = _d_port[i];
    }
    

    ser_pin = _ser_pin;
    ser_port = _ser_port;
    rclk_pin = _rclk_pin;
    rclk_port = _rclk_port;
    srclk_pin = _srclk_pin;
    srclk_port = _srclk_port;
}

uint8_t seg7_char_to_byte(char c)
{
    // アルファベット
    if (isalpha(c))
    {
        if (isupper(c))
            tolower(c);

        return seg_char_alp[(int)c - 'a'];
    }

    // 数字
    if (isdigit(c))
    {
        return seg_char_num[(int)c - '0'];
    }

    // それ以外、記号
    switch (c)
    {
    case ' ':
        return seg_char_sign[0];
        break;
    case '.':
        return seg_char_sign[1];
        break;
    // case 'x':
    //     return seg_char_sign[2];
    case '-':
        return seg_char_sign[3];
        break;
    case '_':
        return seg_char_sign[4];
        break;
    case '=':
        return seg_char_sign[5];
        break;
    // case '°':
    //     return seg_char_sign[6];
    case '"':
        return seg_char_sign[7];
        break;
    }

    // どれにも該当しなかった場合、テストパターンを返す
    return seg_char_sign[2];
}

void seg7_drive(uint8_t digit, uint8_t c)
{
    HAL_GPIO_WritePin(ser_port, ser_pin, 0);
    HAL_GPIO_WritePin(rclk_port, rclk_pin, 0);
    HAL_GPIO_WritePin(srclk_port, srclk_pin, 0);

    // シフトレジスタに溜めてく
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(srclk_port, srclk_pin, 0);
        HAL_GPIO_WritePin(ser_port, ser_pin, READ_BIT(c, 0b10000000 >> i));
        HAL_GPIO_WritePin(srclk_port, srclk_pin, 1);
    }

    // いったん消灯
    for (size_t i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(d_port[i], d_pin[i], 0);
    }

    // レジスタの中身を反映させる
    HAL_GPIO_WritePin(rclk_port, rclk_pin, 1);
    HAL_GPIO_WritePin(rclk_port, rclk_pin, 0);

    HAL_Delay(SEG7_DISPLAY_DELAY);

    // 光らせる
    HAL_GPIO_WritePin(d_port[digit], d_pin[digit], 1);
}

// 文字列、表示される7セグの桁数(注意)、右詰めするか、ピリオドを1つの文字として出力するかどうか
void seg7_print(char str[], uint8_t len, bool isPaddingRight, bool isPeriodIndependent)
{
    if (len > 4)
    {
        len = 4;
    }

    // 右詰め
    uint8_t cursor_start_pos = 0;
    if (isPaddingRight)
    {
        cursor_start_pos = 4 - len;
    }

    uint8_t char_buffer[4] = {0, 0, 0, 0};
    uint8_t str_index = 0;
    for (int c = cursor_start_pos; c < len; c++)
    {
        if (!isPeriodIndependent && str[str_index] == '.')
        {
            // ピリオド
            char_buffer[c - 1] += 1;
            str_index++;
        }

        char_buffer[c] = seg7_char_to_byte(str[str_index]);
        str_index++;
    }

    for (int i = 0; i < 4; i++)
    {
        // 表示
        seg7_drive(i, str[i]);
    }
}
