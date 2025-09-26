#include "IIC3.h"

#define TIM 160
void Delay_ns(uint32_t nus)
{
    uint32_t i;
    while (nus--)
    {
        __NOP();
    }
}

void IIC3_Init(void)
{
    // 开启时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置PC0 PC1 通用推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C3_SCL_Pin | I2C3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 释放总线
    SDA_HIGH;
    SCL_HIGH;
}
void IIC3_Start(void)
{
    SCL_HIGH;
    SDA_HIGH;
    Delay_ns(TIM); // 延时TIMns使SCL高电平稳定
    SDA_LOW;

    Delay_ns(TIM); // 延时获取一个SDA下降沿

    SCL_LOW;

    // 释放SDA,将MCU看成一个从设备
    SDA_HIGH;
    // 保证低电平时间充足
    Delay_ns(TIM);
}

void IIC3_Stop(void)
{
    // 准备产生上升沿
    SDA_LOW;
    SCL_HIGH;
    // 维持准备采样
    Delay_ns(TIM);

    SDA_HIGH;
    Delay_ns(TIM);
}

void IIC3_Send_ack(void)
{
    SDA_LOW;
    // 保证采样时间
    SCL_HIGH;
    Delay_ns(TIM);

    // 释放总线
    SCL_LOW;
    SDA_HIGH;
    Delay_ns(TIM);
}

void IIC3_Send_nack(void)
{
    SDA_HIGH;
    // 保证采样时间
    SCL_HIGH;
    Delay_ns(TIM);

    // 释放总线
    SCL_LOW;
    SDA_HIGH;
    Delay_ns(TIM);
}
ACK_STATE IIC3_Receive_ack(void)
{
    ACK_STATE state;
    // 准备接收确认信号
    SCL_HIGH;
    Delay_ns(TIM);
    if (GET_SDA)
        state = NACK;
    else
        state = ACK;
    // 恢复SCL信号
    SCL_LOW;
    Delay_ns(TIM);
    return state;
}

void IIC3_Send_byte(uint8_t byte)
{
    uint8_t bit;
    // 初始规定SDA高电平释放总线，SCL低电平准备数据
    for (uint8_t i = 0; i < 8; i++)
    {
        // 准备发送数据

        bit = byte & 0x80;
        if (bit)
            SDA_HIGH;
        else
            SDA_LOW;
        SCL_HIGH;
        Delay_ns(TIM);
        // 释放总线SCL低电平准备，SDA释放总线控制
        SCL_LOW;
        SDA_HIGH;
        Delay_ns(TIM);
        // 取下一个低位
        byte <<= 1;
    }
}

uint8_t IIC3_Receive_byte(void)
{
    uint8_t byte;
    for (uint8_t i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCL_HIGH;
        Delay_ns(TIM);
        if (GET_SDA)
        {
            byte |= 0x01;
        }
        SCL_LOW;
        Delay_ns(TIM);
    }
    return byte;
}

uint8_t IIC3_Transmit(uint8_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout){
    IIC3_Start();
    IIC3_Send_byte(dev_addr);
    if(IIC3_Receive_ack()==NACK){
        IIC3_Stop();
        return 1;
    }
    for(uint16_t i=0;i<size;i++){
        IIC3_Send_byte(data[i]);
        if(IIC3_Receive_ack()==NACK){
            IIC3_Stop();
            return 1;
        }
    }
    IIC3_Stop();
    return 0;
}

uint8_t IIC3_Receive(uint8_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout){
    IIC3_Start();
    IIC3_Send_byte(dev_addr | 0x01);
    if(IIC3_Receive_ack()==NACK){
        IIC3_Stop();
        return 1;
    }
    
    for(uint16_t i=0;i<size;i++){
        data[i] = IIC3_Receive_byte();
        IIC3_Send_ack();
    }
    IIC3_Stop();
    return 0;
}
