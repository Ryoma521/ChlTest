/**
  ******************************************************************************
  * @file    node.c
  * @author  Ryoma
  * @version V0.1
  * @date    2-Feb-2015
  * @brief   This file provide the description of node.
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */


#include "node.h"

void DelayUs(uint16 x)
{
  uint8 i;
  while(x--)
  {
    for(i=0;i<4;i++)
      asm("nop");
  }
}

void DelayMs(uint16 x)
{
  while(x--)
  {
    DelayUs(1000);
  }
}



