/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"
#include "main.h"

#include "radio.h"
#include "macro.h"
#include "si4463.h"
#include "si4463_def.h"
#include "radio_config.h"
#include "radio_config_1st.h"
#include "radio_config_2nd.h"
#include "radio_comm.h"
#include "radio_hal.h"
#include "radio_mcu.h"
#include "uart.h"
#include "node.h"
 
#include "hal_types.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t TxBuffer1[]; 
//extern uint8_t TxBuffer2[]; 
extern uint8_t RxBuffer1[];
//extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCounter1;
//extern __IO uint8_t TxCounter2;
extern __IO uint8_t RxCounter1; 
//extern __IO uint8_t RxCounter2;
extern uint8_t NbrOfDataToTransfer1;
//extern uint8_t NbrOfDataToTransfer2;
extern uint8_t NbrOfDataToRead1;
//extern uint8_t NbrOfDataToRead2;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{

}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);

    if(RxCounter1 == NbrOfDataToRead1)
    {
      /* Disable the USARTy Receive interrupt */
      USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    }
  }
  
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, TxBuffer1[TxCounter1++]);

    if(TxCounter1 == NbrOfDataToTransfer1)
    {
      /* Disable the USARTy Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
  }
}

/**
  * @}
  */ 
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer1[RxCounter1++] = USART_ReceiveData(USART2);

    if(RxCounter1 == NbrOfDataToRead1)
    {
      /* Disable the USARTy Receive interrupt */
      USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    }
  }
  
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART2, TxBuffer1[TxCounter1++]);

    if(TxCounter1 == NbrOfDataToTransfer1)
    {
      /* Disable the USARTy Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }    
  }
}

uint8 trx_state =1;//0:tx,1:rx
uint8 PKT_Sent_Flag=0;
extern SEGMENT_VARIABLE( Si446xCmd, union si446x_cmd_reply_union);
void EXTI1_IRQHandler(void)
{
  
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    SI4463_Disable_NIRQ_Int();
 
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearFlag(EXTI_Line1);
    EXTI_ClearITPendingBit(EXTI_Line1);
    
    if (trx_state == 0)//�����ڷ���ģʽ
    {
     /* Read ITs, clear pending ones */
     si446x_get_int_status(0u, 0u, 0u);
    
     /* check the reason for the IT */
     if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT)//δ���ͳɹ��Ļ���Ҫ����������Ŀǰ��ûд
     {
      PKT_Sent_Flag = 1;
      }
    }
    
    
    if (trx_state == 1)//�����ڽ���ģʽ
    {
     DemoApp_Pollhandler_RX();
     }
    
    SI4463_Enable_NIRQ_Int();
  }
  //cont++;
  
}

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
