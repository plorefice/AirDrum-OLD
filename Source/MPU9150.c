/**
  ******************************************************************************
  * @file    MPU9150.c
  * @author  Pietro Lorefice & Daniele Sportillo
  * @version V1.0.0
  * @date    07-October-2014
  * @brief   This file provides a set of functions needed to manage the MPU9150
  *          6DoF MEMS IMU.
  ******************************************************************************
  * @attention
    *
  * (C) COPYRIGHT 2011 Pietro Lorefice / Daniele Sportillo
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "MPU9150.h"
#include "stm32f4xx_it.h"
    
/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup MPU9150
  * @{
  */ 

/** @defgroup MPU9150_Private_TypesDefinitions
  * @{
  */
    
/**
  * @}
  */
	
/** @defgroup MPU9150_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup MPU9150_Private_Macros
  * @{
  */

/* Swap bytes of a 16-bit word */
#define bswap16(X)  (((X & 0x00FF) << 8) | ((X & 0xFF00) >> 8))

/**
  * @}
  */ 
  
/** @defgroup MPU9150_Private_Variables
  * @{
  */ 
	
/* I2C Interface connected to the MPU9150 */
I2C_TypeDef *MPU9150_I2Cx;

/**
  * @}
  */

/** @defgroup MPU9150_Private_FunctionPrototypes
  * @{
  */

static void MPU9150_I2C_Start(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t I2C_Direction);
static void MPU9150_I2C_Stop(I2C_TypeDef *I2Cx);
static void MPU9150_I2C_ReadBuffer(I2C_TypeDef *I2Cx, uint8_t RegAddress, uint8_t *pBuffer, uint8_t nBytes);
static void MPU9150_I2C_WriteBuffer(I2C_TypeDef *I2Cx, uint8_t RegAddress, uint8_t *pData, uint8_t nBytes);

static void MPU9150_LowLevel_Init(void);

/**
  * @}
  */
	
/** @defgroup MPU9150_Public_Functions
  * @{
  */

/**
  * @brief  Set MPU9150 Initialization.
  * @param  MPU9150_InitStruct: pointer to a MPU9150_InitTypeDef structure 
  *         that contains the configuration setting for the MPU9150.
  * @retval None
  */
void MPU9150_Init(MPU9150_InitTypeDef *MPU9150_InitStruct)
{
	uint8_t ctrl = 0x00;
	
	/* Store I2C interface information */
	MPU9150_I2Cx = MPU9150_InitStruct->I2Cx;
	
	/* Initialize hardware peripherals */
	MPU9150_LowLevel_Init();
	
	/* Set MPU9150 Clock Source and power on the device */
	ctrl = (uint8_t)(MPU9150_InitStruct->Clock_Source);
	MPU9150_Write(MPU9150_PWR_MGMT_1_REG_ADDR, &ctrl, 1);
	
	/* Configure the Sample Rate Divider */
	ctrl = (uint8_t)(MPU9150_InitStruct->SampleRate_Divider);
	MPU9150_Write(MPU9150_SMPLRT_DIV_REG_ADDR, &ctrl, 1);
	
	/* Configure the Digital LP Filter */
	ctrl = (uint8_t)(MPU9150_InitStruct->LowPass_Filter);
	MPU9150_Write(MPU9150_CONFIG_REG_ADDR, &ctrl, 1);
	
	/* Configure the Gyroscope */
	ctrl = (uint8_t)(MPU9150_InitStruct->Gyro_FullScale_Range);
	MPU9150_Write(MPU9150_GYRO_CONFIG_REG_ADDR, &ctrl, 1);

	/* Configure the Accelerometer */
	ctrl = (uint8_t)(MPU9150_InitStruct->Accel_FullScale_Range);
	MPU9150_Write(MPU9150_ACCEL_CONFIG_REG_ADDR, &ctrl, 1);
}


/**
  * @brief  Writes one byte to the MPU9150.
  * @param  WriteAddr : MPU9150's internal address to write to.
  * @param  Data: Byte to write.
  * @retval None
  */
void MPU9150_Read(uint8_t RegAddress, uint8_t *pBuffer, uint8_t nBytes)
{
	/* Receive buffer */
	MPU9150_I2C_ReadBuffer(MPU9150_I2Cx, RegAddress, pBuffer, nBytes);
}


/**
  * @brief  Writes one byte to the MPU9150.
  * @param  WriteAddr : MPU9150's internal address to write to.
  * @param  pData: Pointer to the buffer to write.
  * @retval None
  */
void MPU9150_Write(uint8_t RegAddress, uint8_t *pData, uint8_t nBytes)
{
	/* Transmit buffer */
	MPU9150_I2C_WriteBuffer(MPU9150_I2Cx, RegAddress, pData, nBytes);
}


/**
  * @brief  Read the value of the accelerations from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of size int16_t[3].
  * @retval None
  */
void MPU9150_ReadAccel(int16_t *pBuffer)
{
	MPU9150_Read(MPU9150_ACCEL_XOUT_H_REG_ADDR, (uint8_t *)pBuffer, 6);

	pBuffer[0] = (int16_t)(bswap16(pBuffer[0]));
	pBuffer[1] = (int16_t)(bswap16(pBuffer[1]));
	pBuffer[2] = (int16_t)(bswap16(pBuffer[2]));
}


/**
  * @}
  */
	
/** @defgroup MPU9150_Private_Functions
  * @{
  */


/**
  * @brief  Initialize the peripherals to which the MPU9150 is connected.
  * @retval None
  */
static void MPU9150_LowLevel_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStruct;
	I2C_InitTypeDef         I2C_InitStruct;
	
	/* GPIO Configuration */
	
	if (MPU9150_I2Cx == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
	}
	else if (MPU9150_I2Cx == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
	}
	else
	{
		UsageFault_Handler();
	}
	
	/* I2C1 Configuration */
	
	I2C_InitStruct.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1         = 0x00;
	I2C_InitStruct.I2C_Ack                 = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed          = 300000;                           // 100kHz
	I2C_Init(MPU9150_I2Cx, &I2C_InitStruct);
	
	I2C_Cmd(MPU9150_I2Cx, ENABLE);
}


/**
  * @brief  Start I2C bus communication with the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @param  Address : MPU9150's device address.
  * @param  I2C_Direction : Master's communication direction (Transmit/Receive).
  * @retval None
  */
static void MPU9150_I2C_Start(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
	/* Send START configuration */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Wait for ACK */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send 7-bit address */
	I2C_Send7bitAddress(I2Cx, Address, I2C_Direction);
	
	/* Wait for ACK */
	if (I2C_Direction == I2C_Direction_Transmitter)
	{
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) ;
	}
	else
	{
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) ;
	}
}


/**
  * @brief  Stop I2C bus communication with the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @retval None
  */
static void MPU9150_I2C_Stop(I2C_TypeDef *I2Cx)
{
	/* Send STOP configuration */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	/* Wait for ACK */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
}


/**
  * @brief  Read multiple bytes from the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @param  RegAddress : Address of the first egister from which data must be read.
  * @param  pBuffer : Pointer to the buffer in which data will be stored.
  * @param  nBytes : Number of bytes to read.
  * @retval None
  */
static void MPU9150_I2C_ReadBuffer(I2C_TypeDef *I2Cx, uint8_t RegAddress, uint8_t *pBuffer, uint8_t nBytes)
{
	if (nBytes == 0)
		return;
	
	/* Wait until the bus is free */
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	
	/* Send START, Device Address and Write Bit (to send register address) */
	MPU9150_I2C_Start(I2Cx, MPU9150_I2C_ADDR, I2C_Direction_Transmitter);
	
	/* Send register address */
	I2C_SendData(I2Cx, RegAddress);
	
	/* Wait for ACK */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
	
	/* Send second START, Device Address and Read Bit (to receive data) */
	MPU9150_I2C_Start(I2Cx, MPU9150_I2C_ADDR, I2C_Direction_Receiver);
	
	/* Enable ACK for next byte received (Burst mode) */
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	/* While there are still more than one byte to read */
	while ((nBytes--) > 1)
	{
		/* Wait for byte to be ready */
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) ;
		
		/* Receive byte */
		*(pBuffer++) = I2C_ReceiveData(I2Cx);
	}
	
	/* Disable ACK for next byte received (Single mode) */
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	
	/* Wait for byte to be ready */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) ;
	
	/* Send STOP configuration */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	/* Receive last byte */
	*(pBuffer) = I2C_ReceiveData(I2Cx);
}


/**
  * @brief  Write multiple bytes to the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @param  RegAddress : Address of the register in which data must be written.
  * @param  pData : Pointer to the data to write.
  * @param  nBytes : Number of bytes to write.
  * @retval None
  */
static void MPU9150_I2C_WriteBuffer(I2C_TypeDef *I2Cx, uint8_t RegAddress, uint8_t *pData, uint8_t nBytes)
{
	if (nBytes == 0)
		return;
	
	/* Wait until the bus is free */
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	
	/* Send START, Device Address and Write Bit (to send register address) */
	MPU9150_I2C_Start(I2Cx, MPU9150_I2C_ADDR, I2C_Direction_Transmitter);
	
	/* Send register address */
	I2C_SendData(I2Cx, RegAddress);
	
	/* Wait for ACK */
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
	
	/* While there are still bytes to send */
	while ((nBytes--) > 0)
	{
		/* Send single byte */
		I2C_SendData(I2Cx, *(pData++));
		
		/* Wait for ACK */
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
	}
	
	/* Send STOP */
	MPU9150_I2C_Stop(I2Cx);
}

/**
  * @}
  */
	
/**
  * @}
  */ 
  
/**
  * @}
  */
	

/******************* (C) COPYRIGHT 2011 Pietro Lorefice / Daniele Sportillo *****END OF FILE****/
