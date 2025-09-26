#ifndef __IIC3_H
#define __IIC3_H

#include "stm32wbxx.h"
#include "main.h"
#define SCL_HIGH HAL_GPIO_WritePin(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin, GPIO_PIN_SET) 
#define SCL_LOW HAL_GPIO_WritePin(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin, GPIO_PIN_RESET)

#define SDA_HIGH HAL_GPIO_WritePin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin, GPIO_PIN_SET)
#define SDA_LOW HAL_GPIO_WritePin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin, GPIO_PIN_RESET)

#define GET_SDA HAL_GPIO_ReadPin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin)

typedef enum{
    ACK,
    NACK
}ACK_STATE;

void IIC3_Init(void);

void IIC3_Start(void);

void IIC3_Stop(void);

void IIC3_Send_ack(void);

void IIC3_Send_nack(void);

ACK_STATE IIC3_Receive_ack(void);

void IIC3_Send_byte(uint8_t byte);

uint8_t IIC3_Receive_byte(void);

uint8_t IIC3_Transmit(uint8_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout);

uint8_t IIC3_Receive(uint8_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout);




#endif /* __IIC3_H */