/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "hw_config.h"
#include "lwip.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

uint8_t eth_rev = 0;
uint8_t dhcp_state = 0;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void dhcpCheckerTaskThread(void const * argument);
void udpTaskThread(void const *argument);

/* Private function prototypes -----------------------------------------------*/


int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Initialize all configured peripherals */
  init_periph();

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, dhcpCheckerTaskThread, osPriorityBelowNormal, 0, 128);
  osThreadCreate(osThread(defaultTask), NULL); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  while (1)
  {
  }

}

/* StartDefaultTask function */
void dhcpCheckerTaskThread(void const * argument)
{
  MX_LWIP_Init();
  //BSP_ENC28J60_Blink(3);
  while (dhcp_state != DHCP_BOUND)
  {
    checkDhcp();
    osDelay(1);
  }
  MX_LED_DHCP(1);
  osThreadDef(udpTask, udpTaskThread, osPriorityBelowNormal, 0, 128);
  osThreadCreate(osThread(udpTask), NULL); 
  vTaskSuspend(NULL);
}

void udpTaskThread(void const *argument)
{
  struct ip_addr server;
  struct udp_pcb *udpSock;
  struct pbuf *p;
  uint8_t buf[20];
  buf[0] = 'U';
  buf[1] = 'p';
  buf[2] = 't';
  buf[3] = 'i';
  buf[4] = 'm';
  buf[5] = 'e';
  buf[6] = ':';
  
  udpSock = udp_new();
  udp_connect(udpSock, IP_ADDR_ANY, 1020);
  
  p = pbuf_alloc(PBUF_TRANSPORT, 20, PBUF_RAM);
  IP4_ADDR(&server, 192, 168, 1, 130);
  for(;;)
  {
    *((uint32_t*)(buf + 7)) = osKernelSysTick();
    memcpy(p->payload, buf, 20);
    udp_sendto(udpSock, p, &server, 1020);
    osDelay(100);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
