/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 @snkYmkrct
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+

#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_12
#define LED_STATE_ON 1

//--------------------------------------------------------------------+
// BUTTON (tap button on front panel)
//--------------------------------------------------------------------+

#define BUTTON_PORT GPIOB
#define BUTTON_PIN GPIO_PIN_8
#define BUTTON_STATE_ACTIVE 0

//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+

// Flash size of the board
#define BOARD_FLASH_SIZE (64 * 1024)
#define BOARD_FLASH_SECTORS 8

//--------------------------------------------------------------------+
// External QSPI Flash
//--------------------------------------------------------------------+
#define BOARD_QSPI_FLASH_SIZE (4 * 1024 * 1024) // 4MB

//--------------------------------------------------------------------+
// USB UF2
//--------------------------------------------------------------------+

#define USB_VID 0x0483
#define USB_PID 0x5740
#define USB_MANUFACTURER "STM32"
#define USB_PRODUCT "STM32FH7R3V8"

#define UF2_PRODUCT_NAME USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID "STM32FH7R3-dspod"
#define UF2_VOLUME_LABEL "H7R3BOOT"
#define UF2_INDEX_URL "https://github.com/emeb/dspod/tree/main/dspod_h7r3"

#define USB_NO_VBUS_PIN 1
#define BOARD_TUD_RHPORT 0

//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+

#define UART_DEV USART1
#define UART_CLOCK_ENABLE __HAL_RCC_USART1_CLK_ENABLE
#define UART_CLOCK_DISABLE __HAL_RCC_USART1_CLK_DISABLE
#define UART_GPIO_PORT GPIOB
#define UART_GPIO_AF GPIO_AF4_USART1
#define UART_TX_PIN GPIO_PIN_14
#define UART_RX_PIN GPIO_PIN_15

static void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

//--------------------------------------------------------------------+
// RCC Clock - Setup STM32H7R3xx with 400MHz Sysclk from 12Mhz HSE 
// via PLL1. USB OTG FS gets 32MHz refclk via PLL3.
//--------------------------------------------------------------------+
static inline void clock_init(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Configure the system Power Supply */
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    /* Initialization error */
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL1.PLLM = 3;
  RCC_OscInitStruct.PLL1.PLLN = 200;
  RCC_OscInitStruct.PLL1.PLLP = 2;
  RCC_OscInitStruct.PLL1.PLLQ = 2;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLS = 2;
  RCC_OscInitStruct.PLL1.PLLT = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 1;
  RCC_OscInitStruct.PLL3.PLLN = 64;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 32;
  RCC_OscInitStruct.PLL3.PLLR = 2;
  RCC_OscInitStruct.PLL3.PLLS = 2;
  RCC_OscInitStruct.PLL3.PLLT = 2;
  RCC_OscInitStruct.PLL3.PLLFractional = 0;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK4|RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  // Set up non-bus peripherals
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USBOTGFS | RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.UsbOtgFsClockSelection = RCC_USBOTGFSCLKSOURCE_PLL3Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

  /** Enable USB Voltage detector
  */
    HAL_PWREx_EnableUSBVoltageDetector();
	//HAL_PWREx_EnableUSBReg();

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USBPHYC_CLK_ENABLE();
}

//--------------------------------------------------------------------+
// QSPI Flash via XSPI periph. 
//--------------------------------------------------------------------+

static inline void qspi_flash_init(XSPI_HandleTypeDef *hxspi) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/* Enable the XSPIM_P1 interface */
	HAL_PWREx_EnableXSPIM1();

	/* Initializes the peripherals clock */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_XSPI1;
	PeriphClkInit.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/* Peripheral clock enable */
	__HAL_RCC_XSPIM_CLK_ENABLE();
	__HAL_RCC_XSPI1_CLK_ENABLE();

	__HAL_RCC_GPIOP_CLK_ENABLE();
	__HAL_RCC_GPIOO_CLK_ENABLE();
	/**XSPI1 GPIO Configuration
	PP2     ------> XSPIM_P1_IO2
	PP3     ------> XSPIM_P1_IO3
	PO4     ------> XSPIM_P1_CLK
	PP0     ------> XSPIM_P1_IO0
	PP1     ------> XSPIM_P1_IO1
	PO1     ------> XSPIM_P1_NCS2
	PO0     ------> XSPIM_P1_NCS1
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P1;
	HAL_GPIO_Init(GPIOP, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_1|GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P1;
	HAL_GPIO_Init(GPIOO, &GPIO_InitStruct);
	
	/* XSPI1 parameter configuration - timing for GD25Q32C in QSPI */
	hxspi->Instance = XSPI1;
	hxspi->Init.FifoThresholdByte = 1;
	hxspi->Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
	hxspi->Init.MemoryType = HAL_XSPI_MEMTYPE_APMEM;
	hxspi->Init.MemorySize = HAL_XSPI_SIZE_32MB;	// bits!
	hxspi->Init.ChipSelectHighTimeCycle = 2;	// 25ns @ 80MHz (min is 18ns)
	hxspi->Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
	hxspi->Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
	hxspi->Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
	hxspi->Init.ClockPrescaler = 5;	// 80MHz @ 400MHz SYSCLK (max is 80MHz for single mode)
	hxspi->Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
	hxspi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
	hxspi->Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
	hxspi->Init.MaxTran = 240;	// 3us @ 80MHz (max is 3us)
	hxspi->Init.Refresh = 0;
	hxspi->Init.MemorySelect = HAL_XSPI_CSSEL_NCS2;
	if (HAL_XSPI_Init(hxspi) != HAL_OK)
	{
		Error_Handler();
	}
}

#ifdef __cplusplus
}
#endif

#endif
