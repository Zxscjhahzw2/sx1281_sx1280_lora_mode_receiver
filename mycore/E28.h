#ifndef _e28
#define _e28
#include "main.h"

extern uint8_t issendcmd_flag;



void My_E28_2G4M20S_send(uint8_t *buf,uint8_t buf_size);

void My_E28_2G4M20S_receive();

void My_spi_nss_en();//使能nss，他平常的时候是高电平，开始通信的时候是低电平

void My_spi_nss_close();//关闭nss

void My_E28_2G4M20S_init();//初始化，选用lora模式

void My_E28_send(uint8_t *cmd,uint8_t cmd_size);//封装一个e28发送吧

void My_E28_standby();//进入待命模式

void My_E28_writebuffer(uint8_t *buf,uint8_t bufsize);//写入缓冲区

void My_E28_set_effectbuffer(uint8_t buf_size);//告诉lora模块有效的缓冲区

void My_E28_send_data(uint8_t *buf,uint8_t bufsize);

void My_E28_receive_mode();//设置tx，rx引脚mode

void My_E28_receive_cmd();

void My_E28_Enable_Internal_IRQ();

void My_readbuffer(uint8_t offset);

void My_E28_writerxbuffer(uint8_t *buf,uint8_t bufsize);

uint16_t My_E28_GetIrqStatus_Logic();//读取irq寄存器状态，90p

void My_E28_getirqstatus();//用来获取irq寄存器的值

/////////////////////////////////////////////////////////////////////////////////////////////////////
//这里是t系列
void My_E28_init_usartmode();

void My_E28_receive_mode();

void My_E28_standby();

void My_E28_change_to_writecmd();//uart透传模块转换模式,这边模式的切换都是为了适配亿百特的电路设计

void My_E28_change_to_transmitmode();

#endif