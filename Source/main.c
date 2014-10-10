/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
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
#include "main.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis302dl.h"
#include "MPU9150.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
  
__IO uint32_t 	TimingDelay = 0x0;
__IO uint8_t		ProgramExecuting = 0x0;
__IO int16_t    pIMUBuffer[6];

/* Private function prototypes -----------------------------------------------*/
static uint32_t STM_USB_Config(void);
static uint32_t	STM_LIS302DL_Config(void);
static uint32_t MPU9150_Config(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{	
	DMA_InitTypeDef DMA_InitStructure;
	
  /* Initialize LEDs */  
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
	
  /* SysTick end of count event each 2ms */
  SysTick_Config(SystemCoreClock / 1000);
		
	/* USB configuration */
	STM_USB_Config();
	
	/* LIS302DL configuration */
	if (0x0 != STM_LIS302DL_Config())
	{
		Fail_Handler();
	}
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Stream0);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&pIMUBuffer[0]);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 12;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);

  DMA_Cmd(DMA1_Stream0, ENABLE);
	
	/* MPU9150 configuration */
	if (0x0 != MPU9150_Config())
	{
		Fail_Handler();
	}
	
	ProgramExecuting = 0x01;

	while(1)
	{
	}
}


/**
  * @brief  Initializes the USB as VCP.
  * @param  None
  * @retval None
  */
static uint32_t STM_USB_Config(void)
{
  USBD_Init(&USB_OTG_dev,     
            USB_OTG_FS_CORE_ID, 
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
  
  return 0;
}


/**
  * @brief  Initializes the LIS302DL accelerometer.
  * @param  None
  * @retval 0 if successful, 1 otherwise.
  */
static uint32_t STM_LIS302DL_Config(void)
{
	uint8_t bCtrlReg = 0x0;
	
	LIS302DL_InitTypeDef						LIS302DL_InitStruct;
	//LIS302DL_FilterConfigTypeDef		LIS302DL_Filter_InitStruct;
	LIS302DL_InterruptConfigTypeDef	LIS302DL_Interrupt_InitStruct;
	
	/* Set configuration of LIS302DL*/
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_9_2;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);
    
  /* Set configuration of Internal High Pass Filter of LIS302DL*/
  LIS302DL_Interrupt_InitStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_Interrupt_InitStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_XYZ_ENABLE;
  LIS302DL_Interrupt_InitStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_XYZ_DISABLE;
  LIS302DL_InterruptConfig(&LIS302DL_Interrupt_InitStruct);

  /* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
                                                             = 3/100 = 30ms */
  Delay(30);
  
  /* Configure Interrupt control register: enable Click interrupt 1 and 2 */
  bCtrlReg = 0x3F;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CTRL_REG3_ADDR, 1);

  /* Configure Click Threshold on X-Y axis (10 x 0.5g) */
  bCtrlReg = 0x88;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

  /* Configure Click Threshold on Z axis (10 x 0.5g) */
  bCtrlReg = 0x07;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CLICK_THSZ_REG_ADDR, 1);
  
  /* Configure Time Limit */
  bCtrlReg = 0x40;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);
  
  /* Configure Latency */
  bCtrlReg = 0x0C;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);
  
  /* Configure Click Window */
  bCtrlReg = 0x0C;
  LIS302DL_Write(&bCtrlReg, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
	
	/* Self-test */
	LIS302DL_Read(&bCtrlReg, LIS302DL_WHO_AM_I_ADDR, 1);
	
	if (bCtrlReg != 0x3B) return 1;
	
	return 0;
}


/**
  * @brief  Initializes the MPU9150 IMU.
  * @param  None
  * @retval 0 if successful, 1 otherwise.
  */
static uint32_t MPU9150_Config(void)
{
	uint8_t bCtrlReg = 0x0;

	MPU9150_InitTypeDef            MPU9150_InitStruct;
	MPU9150_InterruptConfigTypeDef MPU9150_InterruptInitStruct;
	
	GPIO_InitTypeDef               GPIO_InitStruct;
	EXTI_InitTypeDef               EXTI_InitStruct;
	NVIC_InitTypeDef               NVIC_InitStruct;
	
	/* MPU9150 Configuration */
	MPU9150_InitStruct.I2Cx                  = I2C1;
	MPU9150_InitStruct.Clock_Source          = MPU9150_CLOCK_SRC_GYRO_X_AXIS;
	MPU9150_InitStruct.LowPass_Filter        = MPU9150_LOWPASSFILTER_3;
	MPU9150_InitStruct.SampleRate_Divider    = 99;                             
	MPU9150_InitStruct.Gyro_FullScale_Range  = MPU9150_GYRO_FULLSCALE_2000;
	MPU9150_InitStruct.Accel_FullScale_Range = MPU9150_ACCEL_FULLSCALE_2;
	MPU9150_Init(&MPU9150_InitStruct);
		
	/* Interrupt pin configuration */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin                 = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode                = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed               = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType               = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd                = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
	
	EXTI_InitStruct.EXTI_Line                = EXTI_Line4;
	EXTI_InitStruct.EXTI_Mode                = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger             = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd             = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel          = EXTI4_IRQn;
	NVIC_InitStruct.
		NVIC_IRQChannelPreemptionPriority      = 0x01;
	NVIC_InitStruct.
	  NVIC_IRQChannelSubPriority             = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd       = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	/* MPU9150 Interrupt configuration */
	MPU9150_InterruptInitStruct.Mode         = MPU9150_INTERRUPT_MODE_PUSH_PULL;
	MPU9150_InterruptInitStruct.Level        = MPU9150_INTERRUPT_LEVEL_HIGH;
	MPU9150_InterruptInitStruct.Latched      = MPU9150_INTERRUPT_LATCHED;
	MPU9150_InterruptInitStruct.Sources      = MPU9150_FIFO_ACCEL  |
	                                           MPU9150_FIFO_GYRO_X |
	                                           MPU9150_FIFO_GYRO_Y |
	                                           MPU9150_FIFO_GYRO_Z;
	MPU9150_InterruptConfig(&MPU9150_InterruptInitStruct);
	
	/* WHO_AM_I Test */
	MPU9150_Read(MPU9150_WHO_AM_I_REG_ADDR, &bCtrlReg, 1);
	
	if (bCtrlReg != 0x68) return 1;
	
	return 0;
}


/**
  * @brief  Handler for LIS302DL Timeout events.
  * @param  None
  * @retval None
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
	Fail_Handler();
	return 1;
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
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
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  while(1)
  {
    STM_EVAL_LEDToggle(LED5);
    Delay(50);
  }
}


#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
