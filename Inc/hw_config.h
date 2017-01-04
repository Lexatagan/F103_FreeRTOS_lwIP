/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct load_counter
{
  uint32_t cnt;
  uint32_t result;
}cpu_LoadCounterTypeDef;
/* Exported constants --------------------------------------------------------*/

/* SPI */
#define ETHERNET_SPI                    SPI2
#define __ETHSPI_CLK_ENABLE()           __SPI2_CLK_ENABLE()
#define __ETHSPI_CLK_DISABLE()          __SPI2_CLK_DISABLE()
#define __ETHSPI_PORT_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define __ETHSPI_PORT_CLK_DISABLE()     __GPIOB_CLK_DISABLE()
#define ETHSPI_PORT                     GPIOB
#define ETHSPI_CS_PIN                   GPIO_PIN_12
#define ETHSPI_SCK_PIN                  GPIO_PIN_13
#define ETHSPI_MISO_PIN                 GPIO_PIN_14
#define ETHSPI_MOSI_PIN                 GPIO_PIN_15
#define LED_PORT                        GPIOA
#define LED_DHCP                        GPIO_PIN_4        

/* NVIC */
#define SYSTICK_PREEMPT_PRIORITY        2       //Low

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern cpu_LoadCounterTypeDef cpuLoad;
/* Exported functions ------------------------------------------------------- */

void init_periph(void);
void Error_Handler(void);
void SystemClock_Config(void);
void MX_LED_DHCP(uint8_t state);

#endif /* __HW_CONFIG_H */
