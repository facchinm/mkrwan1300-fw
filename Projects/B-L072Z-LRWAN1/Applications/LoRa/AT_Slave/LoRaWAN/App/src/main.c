 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
#include "hw.h"
#include "low_power_manager.h"
#include "timeServer.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "lora.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/**
 * When fast wake up is enabled, the mcu wakes up in ~20us and
 * does not wait for the VREFINT to be settled. THis is ok for
 * most of the case except when adc must be used in this case before
 * starting the adc, you must make sure VREFINT is settled
 */
#define ENABLE_FAST_WAKEUP

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LoraRxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/**
 * Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK};

                                    
                                    
/* Private functions ---------------------------------------------------------*/

static bool goDumb() {
  // Setup GPIOPB12 as input
  GPIO_InitTypeDef initStruct={0};
  initStruct.Mode = GPIO_MODE_INPUT;
  initStruct.Pull = GPIO_PULLUP;
  initStruct.Speed = GPIO_SPEED_HIGH;

  HW_GPIO_Init( GPIOB, GPIO_PIN_12, &initStruct );

  return HW_GPIO_Read(GPIOB, GPIO_PIN_12) == 0;
}

void setupPassthrough() {
 // input pins
 GPIO_InitTypeDef initStruct={0};
 initStruct.Mode = GPIO_MODE_INPUT;
 initStruct.Pull = GPIO_NOPULL;
 initStruct.Speed = GPIO_SPEED_HIGH;

 HW_GPIO_Init( GPIOB, GPIO_PIN_13, &initStruct );
 HW_GPIO_Init( GPIOA, GPIO_PIN_3, &initStruct );
 HW_GPIO_Init( GPIOA, GPIO_PIN_6, &initStruct );
 HW_GPIO_Init( GPIOB, GPIO_PIN_4, &initStruct );

 // output pins
 initStruct.Mode = GPIO_MODE_OUTPUT_PP;
 initStruct.Pull = GPIO_NOPULL;

 HW_GPIO_Init( GPIOB, GPIO_PIN_3, &initStruct );
 HW_GPIO_Init( GPIOA, GPIO_PIN_7, &initStruct );
 HW_GPIO_Init( GPIOA, GPIO_PIN_2, &initStruct );
#ifdef USE_DIO0_IRQ
 HW_GPIO_Init( GPIOB, GPIO_PIN_12, &initStruct );
#endif
 HW_GPIO_Init( GPIOB, GPIO_PIN_15, &initStruct );

 // reset and SS pins
 SX1276Reset();
 HW_GPIO_Init( RADIO_NSS_PORT, RADIO_NSS_PIN, &initStruct );
#ifdef USE_DIO0_IRQ
 HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
#endif
}

static inline void runPassthrough() {
 // PB13 -> PB3
 // PA3 -> PA7
 // PA6 -> PA2
 // PB12 -> PB15

 __asm__ volatile (
   "LDR R0, =0x50000014\n\t" //GPIOA_BSRR
   "LDR R1, =0x50000414\n\t" //GPIOB_BSRR
   "LDR R2, =0x50000010\n\t" //GPIOA_IDR
   "LDR R3, =0x50000410\n\t" //GPIOB_IDR

   "MOV R6, #1\n\t"
   "dumb:\n\t"

   "LDR R7, [R3]\n\t"

   //"LSR R4, R7, #13\n\t"
   //"AND R4, R4, R6\n\t"
   //"LSL R4, R4, #3\n\t"
   "LSR R4, R7, #10\n\t"
   "STR R4, [R1]\n\t"

   "LSR R4, R7, #12\n\t"
   "AND R4, R4, R6\n\t"
   "LSL R5, R4, #15\n\t"

   "LDR R7, [R2]\n\t"

   "LSR R4, R7, #3\n\t"
   "AND R4, R4, R6\n\t"
   "LSL R4, R4, #7\n\t"
   "ORR R5, R5, R4\n\t"

   "LSR R4, R7, #6\n\t"
   "AND R4, R4, R6\n\t"
   "LSL R4, R4, #2\n\t"
   "ORR R5, R5, R4\n\t"

   "STR R5, [R0]\n\t"

   "B dumb\n\t"
 );
}

LoRaMacRegion_t globalRegion = LORAMAC_REGION_EU868;

void LORA_ReInit() {
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
    /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();

  /* Read SS input (GPIOPB12); if low, enter dumb mode */
  if (goDumb()) {
    setupPassthrough();
    while (1) {
      runPassthrough();
    }
  }

  //HW_GPIO_DeInit( GPIOB, GPIO_PIN_12);
  HW_GpioInit();

  /* Configure the hardware*/
  HW_Init();

  /* Configure Debug mode */
  DBG_Init();
  
  /* USER CODE BEGIN 1 */
  CMD_Init();
  /*Disable standby mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
  
  PRINTF("+EVENT=0,0");
  /* USER CODE END 1 */

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

  /* main loop*/
  while (1)
  {
    /* Handle UART commands */
    CMD_Process();
    
    LoRaMacProcess( );
    /*
     * low power section
     */
    DISABLE_IRQ();
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();

    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}


static void LoraRxData(lora_AppData_t *AppData)
{
   set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
   // TODO: why did we add this?
   //at_Receive(NULL);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  Error_Handler();
}
#endif

static void LORA_HasJoined( void )
{
  PRINTF("+EVENT=1,1\r");
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );
}

static void LORA_TxNeeded ( void )
{
  PRINTF("Network Server is asking for an uplink transmission\n\r");
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
