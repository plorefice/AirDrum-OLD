/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
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
#include "stm32f4xx_it.h"
#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis302dl.h"
#include "MPU9150.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LIS302DL_PENDING_IRQ	0x40
#define LIS302DL_CLICK_X			0x01
#define LIS302DL_CLICK_Y			0x04
#define LIS302DL_CLICK_Z			0x10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t	TimingDelay;
extern __IO uint8_t		ProgramExecuting;
extern __IO	int32_t		AccelerationValue[3];
extern __IO uint8_t		SingleClickXDetect;
extern __IO uint8_t		SingleClickYDetect;
extern __IO uint8_t		SingleClickZDetect;
						uint8_t		ClickReg = 0x0;

/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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
  * @brief  This function handles PendSVC exception.
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
//	uint32_t Counter = 0x0;
	int16_t  pRawData[3];
	float		 pData[3];
	
	if (TimingDelay != 0)
	{
		TimingDelay_Decrement();
	}
	
	if (ProgramExecuting)
	{
		MPU9150_ReadFIFO(pRawData);
		
		MPU9150_ReadAccel(pRawData);
		
		pData[0] = pRawData[0] / MPU9150_ACCEL_SENSITIVITY_2;
		pData[1] = pRawData[1] / MPU9150_ACCEL_SENSITIVITY_2;
		pData[2] = pRawData[2] / MPU9150_ACCEL_SENSITIVITY_2;

		VCP_DataTx((uint8_t *)pData, sizeof(pData));
		
		STM_EVAL_LEDToggle(LED6);
	}

//	else if (ProgramExecuting && ((++Counter) == 20))
//	{
//		uint8_t	i;
//		int32_t AccelerationTmp[3];
//		
//		LIS302DL_ReadACC((int32_t *)AccelerationTmp);
//		
//		for (i = 0; i < 3; i++)
//		{
//			AccelerationValue[i] = (AccelerationTmp[i] > AccelerationValue[i]) ?
//															AccelerationTmp[i] : AccelerationValue[i];
//		}
//		
//		LIS302DL_Read(&ClickReg, LIS302DL_CLICK_SRC_REG_ADDR, 1);
//		
//		if (ClickReg & LIS302DL_PENDING_IRQ)
//		{
//			if (ClickReg & LIS302DL_CLICK_X) SingleClickXDetect = 0x01;
//			if (ClickReg & LIS302DL_CLICK_Y) SingleClickYDetect = 0x01;
//			if (ClickReg & LIS302DL_CLICK_Z) SingleClickZDetect = 0x01;
//		}
//		
//		Counter = 0x0;
//	}
}

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
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
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		DrumKitChangeState();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
	/* Reset SLEEPDEEP and SLEEPONEXIT bits */
	SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

	/* After wake-up from sleep mode, reconfigure the system clock */
	SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
