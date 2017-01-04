//******************************************************************************
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "hw_config.h"
#include "time.h"

/* External variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
cpu_LoadCounterTypeDef cpuLoad;
SD_HandleTypeDef hsd;
SPI_HandleTypeDef hethspi;
DMA_HandleTypeDef hdma_sdio;
HAL_SD_CardInfoTypedef SDCardInfo;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SDIO_SD_Init(void);
void MX_ETHSPI_Init(void);

/* Peripheral Configuration */
void init_periph(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETHSPI_Init();

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

/* Configure pins */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __ETHSPI_PORT_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = LED_DHCP;                                               //TODO
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* Ethernet SPI init function */
void MX_ETHSPI_Init(void)
{
  hethspi.Instance = ETHERNET_SPI;
  hethspi.Init.Mode = SPI_MODE_MASTER;
  hethspi.Init.Direction = SPI_DIRECTION_2LINES;
  hethspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hethspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  hethspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  hethspi.Init.NSS = SPI_NSS_SOFT;
  hethspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hethspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hethspi.Init.TIMode = SPI_TIMODE_DISABLED;
  hethspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hethspi.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hethspi);
}

void MX_LED_DHCP(uint8_t state)
{
  if (state)
    HAL_GPIO_WritePin(LED_PORT, LED_DHCP, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LED_PORT, LED_DHCP, GPIO_PIN_RESET);    
}