/**
  ******************************************************************************
  * @file    USART/Printf/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "il_hw_init.h"
#include "il_usart.h"

#include "radio_mcu.h"
#include "radio.h"
#include "macro.h"
#include "si4463.h"
#include "si4463_def.h"
#include "radio_config.h"
#include "radio_comm.h"
#include "node.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t RxBuffer[]={0x01};

ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t ADCConvertedValue[2];

extern SEGMENT_VARIABLE(Radio_Configuration_Data_Array[], U8);

extern SEGMENT_VARIABLE(RadioConfiguration, tRadioConfiguration);

extern SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration) ;

extern SEGMENT_VARIABLE( Si446xCmd, union si446x_cmd_reply_union);

extern SEGMENT_VARIABLE(fixRadioPacket[RADIO_MAX_PACKET_LENGTH], U8);

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);
extern __IO uint8_t RxCounter1;
void PwmOut(uint8_t time);
void RelayOn();
void RelayOff();
void DMAADCConfig();
/* Private functions ---------------------------------------------------------*/

//void SysTick_Config(void)
//{
//  //设置失能SysTick定时器 
//  //SysTick_CounterCmd(SysTick_Counter_Disable); 
//  //设置失能SysTick中断 
//  //SysTick_ITConfig(DISABLE); 
//  //设置SysTick的时钟源为AHB时钟 
//  //SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
//  //设置重装载值，由于SysTick是AHB时钟，即72MHz，所以重装载值设置为72000，即每1ms重新装载一次 
//  //SysTick_SetReload(72000); 
//  //设置SysTick定时器中断优先级 
//  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 0, 0); 
//  //设置使能SysTick中断 
//  SysTick_ITConfig(ENABLE); 
//}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */   
  
    /* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
    
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm3.h file.

    4. To adjust the SysTick time base, use the following formula:
                            
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
    
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
//  if (SysTick_Config(SYSCLK_FREQ_72MHz/100))
//  { 
//    /* Capture error */ 
//    while (1);
//  }
  
  //SysTick_Config();
  
  RCC_ClocksTypeDef RCC_ClockFreq;
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  Il_Hw_Init();  
  Init_SI4463_Pin();
  vRadio_Init();   
  vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);
  while(1)
  {
#if 0
    DelayMs(1000);
    /* Set PB5 and PB6 */
    GPIOB->BSRR = 0x00000060;
    vRadio_StartTx_Variable_Packet(0u,RxBuffer,1);
    DelayMs(1000);
    /* Reset PB5 and PB6 */
    GPIOB->BRR  = 0x00000060; 
#endif
    
    GPIOB->BSRR = 0x00000040;    
    DelayMs(1000);
    /* Reset PB5 and PB6 */
    GPIOB->BRR  = 0x00000040;
    DelayMs(1000);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  PWM_OUT
  * @param  None
  * @retval None
  */
void PwmOut(uint8_t time)
{
  /* Set PD2*/
  GPIOD->BSRR = 0x00000004;
  Delay(time);
  /* Reset PD2 */
  GPIOD->BRR  = 0x00000004;
  Delay(5-time);
}

/**
  * @brief  RELAY_ON
  * @param  None
  * @retval None
  */
void RelayOn()
{
  /* Set PA1*/
  GPIOA->BSRR = 0x00000002;
}

/**
  * @brief  RELAY_OFF
  * @param  None
  * @retval None
  */
void RelayOff()
{
  /* Reset PA1 */
  GPIOA->BRR  = 0x00000002;
}

/**
  * @}
  */ 
void DMAADCConfig()
{
 /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel12,13 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_55Cycles5); //Current
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_55Cycles5); //Voltage

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
