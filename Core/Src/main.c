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
#include "imrc_seg7_x4.h"
#include "stm32f3xx_hal_adc.h"
#include "stm32f3xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_R1_OHM 1000
#define ADC_R2_OHM 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void pwr_relay(uint8_t relay_index, uint8_t state);
void dbg_led(uint8_t led_index, uint8_t state);
uint8_t dbg_sw(uint8_t sw_index);

void __io_putchar(uint8_t ch)
{
    HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t adc_dma_buffer[2];

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
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_CAN_Init();
    /* USER CODE BEGIN 2 */

    uint16_t seg7_digit_pin[4] = {
        SEG_DIG_0_Pin,
        SEG_DIG_1_Pin,
        SEG_DIG_2_Pin,
        SEG_DIG_3_Pin,
    };

    GPIO_TypeDef *seg7_digit_port[4] = {
        SEG_DIG_0_GPIO_Port,
        SEG_DIG_1_GPIO_Port,
        SEG_DIG_2_GPIO_Port,
        SEG_DIG_3_GPIO_Port,
    };

    // 7セグ 初期化
    seg7_init(seg7_digit_pin, seg7_digit_port, SEG_SER_Pin, SEG_SER_GPIO_Port, SEG_RCLK_Pin, SEG_RCLK_GPIO_Port, SEG_SRCLK_Pin, SEG_SRCLK_GPIO_Port);

    pwr_relay(0, 0);
    pwr_relay(1, 0);

    dbg_led(0, 1);
    dbg_led(1, 1);
    dbg_led(2, 1);

    // HAL_Delay(500);
    for (int i = 0; i < 100; i++)
    {
        // char boot_7seg[] = "417-";
        char boot_7seg[] = "volt";
        seg7_print(boot_7seg, 4, false, false);
    }
    

    dbg_led(0, 0);
    dbg_led(1, 0);
    dbg_led(2, 0);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);

    setbuf(stdout, NULL);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        char seg7_buffer[8];
        // seg7_print(seg7_buffer, 4, false, false);

        HAL_ADC_Start(&hadc1);

        HAL_ADC_PollForConversion(&hadc1, 1000);
        uint32_t adc_voltage_0 = HAL_ADC_GetValue(&hadc1);
        float f_vol_0 = (float)((float)adc_voltage_0 / (float)4096) * 3.3;
        f_vol_0 *= 8.04878f;
        f_vol_0 -= 1.0f;

        if(f_vol_0 < 0){
            f_vol_0 = 0.0f;
        }

        if(f_vol_0 <= 19.5f)
        {
            pwr_relay(0, 1);
            dbg_led(2, 1);
            dbg_led(0, 0);
        } else {
            pwr_relay(0, 0);
            dbg_led(2, 0);
            dbg_led(0, 1);
        }

        /* HAL_ADC_PollForConversion(&hadc1, 10);
        uint32_t adc_voltage_1 = HAL_ADC_GetValue(&hadc1); */

        printf("%.3fv\n", f_vol_0);

        if (f_vol_0 < 10.0f)
        {
            snprintf(seg7_buffer, 8, "%.2f", f_vol_0); // 小数点第1位まで（結果: "12.2"）
        }
        else
        {
            snprintf(seg7_buffer, 8, "%.1f", f_vol_0); 
        }

        seg7_buffer[4] = 'v';
        seg7_print(seg7_buffer, 4, false, false);

        // HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
        // printf("CHANNEL 0: %d\n", adc_voltage_0);

        // printf("CHANNEL 0: %d, CHANNEL 1: %d\n", adc_voltage_0, adc_voltage_1);
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN;
    hcan.Init.Prescaler = 16;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN_Init 2 */

    /* USER CODE END CAN_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
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
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RELAY_0_Pin | RELAY_1_Pin | LED_0_Pin | LED_1_Pin | SEG_SER_Pin | SEG_RCLK_Pin | SEG_SRCLK_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED_2_Pin | SEG_DIG_0_Pin | SEG_DIG_1_Pin | SEG_DIG_2_Pin | SEG_DIG_3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : BZ_Pin */
    GPIO_InitStruct.Pin = BZ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BZ_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RELAY_0_Pin RELAY_1_Pin LED_0_Pin LED_1_Pin
                             SEG_SER_Pin SEG_RCLK_Pin SEG_SRCLK_Pin */
    GPIO_InitStruct.Pin = RELAY_0_Pin | RELAY_1_Pin | LED_0_Pin | LED_1_Pin | SEG_SER_Pin | SEG_RCLK_Pin | SEG_SRCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SW_0_Pin */
    GPIO_InitStruct.Pin = SW_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SW_1_Pin */
    GPIO_InitStruct.Pin = SW_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_2_Pin SEG_DIG_0_Pin SEG_DIG_1_Pin SEG_DIG_2_Pin
                             SEG_DIG_3_Pin */
    GPIO_InitStruct.Pin = LED_2_Pin | SEG_DIG_0_Pin | SEG_DIG_1_Pin | SEG_DIG_2_Pin | SEG_DIG_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void pwr_relay(uint8_t relay_index, uint8_t state)
{
    uint16_t relay_pin[2] = {
        RELAY_0_Pin,
        RELAY_1_Pin};
    GPIO_TypeDef *relay_port[2] = {
        RELAY_0_GPIO_Port,
        RELAY_1_GPIO_Port};

    HAL_GPIO_WritePin(relay_port[relay_index], relay_pin[relay_index], state);
}

void dbg_led(uint8_t led_index, uint8_t state)
{
    uint16_t led_pin[3] = {
        LED_0_Pin,
        LED_1_Pin,
        LED_2_Pin,
    };
    GPIO_TypeDef *led_port[3] = {
        LED_0_GPIO_Port,
        LED_1_GPIO_Port,
        LED_2_GPIO_Port};

    HAL_GPIO_WritePin(led_port[led_index], led_pin[led_index], state);
}

uint8_t dbg_sw(uint8_t sw_index)
{
    GPIO_TypeDef *sw_port[3] = {
        SW_0_GPIO_Port,
        SW_1_GPIO_Port};

    uint16_t sw_pin[3] = {
        SW_0_Pin,
        SW_1_Pin};

    return !(HAL_GPIO_ReadPin(sw_port[sw_index], sw_pin[sw_index]));
}

float pwr_getVoltage(uint8_t channel)
{
    float voltage = 0.0;

    // ADCで分圧された値を取得

    voltage = voltage * (float)(ADC_R1_OHM / (ADC_R1_OHM + ADC_R2_OHM));
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
