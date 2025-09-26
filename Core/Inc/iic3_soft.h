#ifndef IIC3_SOFT_H
#define IIC3_SOFT_H
/* 软件模拟 I2C (IIC3) 带错误检查
 * 适配 STM32WB55 (或其他 STM32) GPIO。用户需在本文件底部区域完成引脚和延时宏配置。
 */
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stddef.h>

/* ===================== 错误与结果定义 ===================== */
typedef enum {
    IIC3_OK = 0,
    IIC3_ERR_BUS_BUSY,           /* 上电或操作前 SDA 被拉低 */
    IIC3_ERR_START_TIMEOUT,      /* 产生 START 时 SDA/SCL 未达到期望电平 */
    IIC3_ERR_REPEATED_START_FAIL,/* 重复 START 失败 */
    IIC3_ERR_ACK_TIMEOUT,        /* 等待 ACK 超时 */
    IIC3_ERR_NACK,               /* 收到 NACK */
    IIC3_ERR_SCL_STRETCH_TIMEOUT,/* 从设备时钟拉伸超时 */
    IIC3_ERR_ARB_LOST,           /* 仲裁丢失（多主场景) */
    IIC3_ERR_PARAM,              /* 参数错误 */
    IIC3_ERR_TIMEOUT,            /* 通用超时 */
    IIC3_ERR_BUS_RECOVER_FAIL    /* 总线恢复失败 */
} IIC3_Result;

/* 详细状态（可选使用，不需要可忽略该结构） */
typedef struct {
    IIC3_Result last_error;
    uint8_t step;        /* 代码内部阶段标记 */
    uint8_t bit_index;   /* 正在处理的 bit 下标 */
    uint8_t retries;     /* 当前已尝试的重试次数 */
} IIC3_StatusDetail;

/* ===================== 用户可调参数结构 ===================== */
typedef struct {
    uint16_t delay_half_cycle_ticks; /* 半周期延时（基于 Delay 函数的单位）。100kHz 时一个完整周期10us => 半周期5us */
    uint16_t scl_stretch_timeout;    /* SCL 等待为高的最大循环次数 */
    uint16_t ack_timeout;            /* 等待 ACK 低电平的最大循环次数 */
    uint16_t bus_free_check;         /* 上电或开始前检测 SDA/SCL 空闲的循环次数 */
    uint8_t  max_retries;            /* 发送地址或数据出现 NACK 的最大重试次数 */
} IIC3_Config;

/* 句柄 */
typedef struct {
    IIC3_Config cfg;
    IIC3_StatusDetail status; /* 可选状态 */
} IIC3_Handle;

/* ===================== 公共 API ===================== */
void IIC3_Init(IIC3_Handle *hiic, const IIC3_Config *cfg);
IIC3_Result IIC3_BusRecover(IIC3_Handle *hiic);
IIC3_Result IIC3_Start(IIC3_Handle *hiic);
IIC3_Result IIC3_RepeatedStart(IIC3_Handle *hiic);
IIC3_Result IIC3_Stop(IIC3_Handle *hiic);
IIC3_Result IIC3_WriteByte(IIC3_Handle *hiic, uint8_t data);
IIC3_Result IIC3_ReadByte(IIC3_Handle *hiic, uint8_t *data, uint8_t send_ack);

/* 高层封装：写设备寄存器 / 读设备寄存器 */
IIC3_Result IIC3_WriteReg(IIC3_Handle *hiic, uint8_t dev7bitAddr, uint8_t reg, const uint8_t *buf, size_t len);
IIC3_Result IIC3_ReadReg(IIC3_Handle *hiic, uint8_t dev7bitAddr, uint8_t reg, uint8_t *buf, size_t len);

/* 获取最后错误 */
static inline IIC3_Result IIC3_GetLastError(IIC3_Handle *hiic){ return hiic->status.last_error; }

/* ============= 用户需要提供的底层 GPIO & 延时宏 (根据工程调整) ============= */
/* 以下示例假设：SCL = PB8, SDA = PB9，用户需要用正确的寄存器或 HAL GPIO 操作替换。*/
/* 如果使用 HAL，请包含 stm32wbxx_hal.h，并用 HAL_GPIO_WritePin / HAL_GPIO_ReadPin 实现。 */

#ifndef IIC3_GPIO_DEFINED
#include "stm32wbxx_hal.h"
#define IIC3_SCL_GPIO_Port GPIOB
#define IIC3_SCL_Pin       GPIO_PIN_8
#define IIC3_SDA_GPIO_Port GPIOB
#define IIC3_SDA_Pin       GPIO_PIN_9

/* 设置 SDA 为输出/输入: 若使用开漏+外部上拉，可只配置一次，此处演示简单控制 */
static inline void IIC3_SDA_Out(void){
    GPIO_InitTypeDef gi = {0};
    gi.Pin = IIC3_SDA_Pin; gi.Mode = GPIO_MODE_OUTPUT_OD; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(IIC3_SDA_GPIO_Port,&gi);
}
static inline void IIC3_SDA_In(void){
    GPIO_InitTypeDef gi = {0};
    gi.Pin = IIC3_SDA_Pin; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(IIC3_SDA_GPIO_Port,&gi);
}
static inline void IIC3_SCL_Out(void){
    GPIO_InitTypeDef gi = {0};
    gi.Pin = IIC3_SCL_Pin; gi.Mode = GPIO_MODE_OUTPUT_OD; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(IIC3_SCL_GPIO_Port,&gi);
}

#define IIC3_SCL_HIGH() HAL_GPIO_WritePin(IIC3_SCL_GPIO_Port, IIC3_SCL_Pin, GPIO_PIN_SET)
#define IIC3_SCL_LOW()  HAL_GPIO_WritePin(IIC3_SCL_GPIO_Port, IIC3_SCL_Pin, GPIO_PIN_RESET)
#define IIC3_SDA_HIGH() HAL_GPIO_WritePin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin, GPIO_PIN_SET)
#define IIC3_SDA_LOW()  HAL_GPIO_WritePin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin, GPIO_PIN_RESET)
#define IIC3_READ_SDA() (HAL_GPIO_ReadPin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin)==GPIO_PIN_SET)
#define IIC3_READ_SCL() (HAL_GPIO_ReadPin(IIC3_SCL_GPIO_Port, IIC3_SCL_Pin)==GPIO_PIN_SET)

/* 简单延时：需用户实现一个粗略微秒延时。可以用 DWT 或 HAL_Delay 微调。
 * delay_ticks 参数为 cfg.delay_half_cycle_ticks 传入值。
 */
static inline void IIC3_DelayTicks(uint16_t ticks){
    volatile uint16_t t = ticks; while(t--) __NOP();
}
#endif /* IIC3_GPIO_DEFINED */

#ifdef __cplusplus
}
#endif
#endif /* IIC3_SOFT_H */
