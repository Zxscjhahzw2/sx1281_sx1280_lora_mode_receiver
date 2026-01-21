#include "E28.h"

extern UART_HandleTypeDef huart1;

uint8_t issendcmd_flag=0;//0表示没有发命令，1表示在发命令，用于区分AUX是接受到了命令还是外部来的数据

extern UART_HandleTypeDef huart1;
extern float temp ;  
extern float humidity ;
extern SPI_HandleTypeDef hspi1;

void My_E28_send(uint8_t *cmd,uint8_t cmd_size)//封装一个e28发送吧
{
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1,cmd,cmd_size,100);
    My_spi_nss_close();
    HAL_Delay(5);
    
}

void My_E28_standby()//进入待命模式
{
    uint8_t buf[2];
    buf[0]=0x80;
    buf[1]=0x00;
    My_E28_send(buf,2);
}
void My_readbuffer(uint8_t offset){
	uint8_t buf[33]={0x1b,offset,0x00};
	uint8_t rx[33]={0};
    My_spi_nss_en();
	HAL_Delay(5);
HAL_SPI_TransmitReceive(&hspi1, buf, rx, 33, 100);
    My_spi_nss_close();
    HAL_Delay(20);


}

void My_E28_writerxbuffer(uint8_t *buf,uint8_t bufsize)//写入缓冲区
{
    uint8_t op_code = 0x1A;
    uint8_t offset = 0x00;

    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, &op_code, 1, 100); // 发指令
    HAL_SPI_Transmit(&hspi1, &offset, 1, 100);  // 发偏移量
    HAL_SPI_Transmit(&hspi1, buf, bufsize, 100); // 连续发数据，中途不准拉高NSS
    My_spi_nss_close();
    HAL_Delay(20);

}


void My_E28_writebuffer(uint8_t *buf,uint8_t bufsize)//写入缓冲区,tx
{
    uint8_t op_code = 0x1A;
    uint8_t offset = 0x80;

    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, &op_code, 1, 100); // 发指令
    HAL_SPI_Transmit(&hspi1, &offset, 1, 100);  // 发偏移量
    HAL_SPI_Transmit(&hspi1, buf, bufsize, 100); // 连续发数据，中途不准拉高NSS
    My_spi_nss_close();
    HAL_Delay(20);

}
void My_E28_set_effectbuffer(uint8_t buf_size)//告诉lora模块有效的缓冲区
{
    uint8_t update_pkt[] = {0x8C, 0x0C, 0x00, buf_size, 0x00, 0x40, 0x00, 0x00};
    My_E28_send(update_pkt,8);
}

void My_E28_send_data(uint8_t *buf,uint8_t bufsize)
{ uint8_t cmd_tx[] = {0x83, 0x00, 0x00, 0x00};//连续tx模式
    My_E28_standby(); // 1. 先待命，确保状态机干净
   
    My_E28_writebuffer(buf,bufsize);
    My_E28_set_effectbuffer(bufsize);
    My_E28_send(cmd_tx,4);
}

void My_E28_receive_cmd(){
    My_E28_receive_mode();
    // uint8_t rx_cmd[]={0x82,0x03,0xff,0xff};//连续接受模式,75p，gemini说第二个参数要改ff，但手册里没看到

    // My_spi_nss_en();
    // HAL_SPI_Transmit(&hspi1,rx_cmd,4,100);
    // My_spi_nss_close();
    // HAL_Delay(20);

    // 0x82, 0x00, 0x00, 0x00 代表单次接收，直到收到为止
uint8_t rx_cmd_single[] = {0x82, 0x00, 0x00, 0x00};//单次接受模式
My_E28_send(rx_cmd_single, 4);


}


void My_E28_getirqstatus()//用来获取irq寄存器的值
{
    // uint8_t buf[]={0x15,0x00,0x00,0x00};
    // uint8_t irq_res[4] = {0};
    // My_spi_nss_en();
    // HAL_SPI_Transmit(&hspi1,buf,4,100);
    // My_spi_nss_close();
    // HAL_Delay(20);    
}


uint16_t My_E28_GetIrqStatus_Logic() {
    // 0x15 是 GetIrqStatus 的指令码
    // 我们需要 4 个字节：1个指令 + 1个虚拟字节 + 2个字节接数据
    uint8_t tx_buf[4] = {0x15, 0x00, 0x00, 0x00};//91p
    uint8_t rx_buf[4] = {0};

    My_spi_nss_en();
	HAL_Delay(5);
    // 使用全双工交换数据
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 4, 100);
    My_spi_nss_close();

    // 根据我们之前的讨论：
    // rx_buf[0] 和 [1] 是状态字节 (Status)
    // rx_buf[2] 是 IRQ 高 8 位
    // rx_buf[3] 是 IRQ 低 8 位
    uint16_t irq_val = (uint16_t)(rx_buf[2] << 8) | rx_buf[3];
    
    return irq_val;
}


void My_E28_receive_mode(){
    HAL_GPIO_WritePin(TX_EN_GPIO_Port,TX_EN_Pin,0);
    HAL_GPIO_WritePin(RX_EN_GPIO_Port,RX_EN_Pin,1);
}

void My_E28_Enable_Internal_IRQ() {
    // 0x8d 是标准 SX1280 的 SetDioIrqParams 操作码
    // 参数1-2: 0xFFFF (使能所有内部中断记录)
    // 参数3-8: 全 0 (不把中断输出到没接线的 DIO 引脚)
    uint8_t irq_cmd[] = {0x8d, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, irq_cmd, 9, 100);
    My_spi_nss_close();
    HAL_Delay(5);
}


void My_E28_2G4M20S_init() 
{

        //上电校准
    uint8_t standby[] = {0x80, 0x00}; // STDBY_RC
    My_E28_send( standby, 2);
    HAL_Delay(20);
    // 准备指令数组
    uint8_t cmd_type[] = {0x8a, 0x01};                               
    uint8_t cmd_freq[] = {0x86, 0xb8, 0x9d, 0x89};                   
    uint8_t cmd_txp[]  = {0x8e, 0x1f, 0x15};                         
    uint8_t cmd_base[] = {0x8f, 0x80, 0x00};                         
    uint8_t cmd_mod[]  = {0x8b, 0x70, 0x0a, 0x01};                   
    uint8_t cmd_pkt[]  = {0x8c, 0x0c, 0x00, 0x80, 0x00, 0x40, 0x00, 0x00}; 

    // --- 逐条发送，每条必须独立拉高 NSS 并延时 ---

    // 1. SetPacketType
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_type, 2, 100);
    My_spi_nss_close();
    HAL_Delay(20); // 盲等 5ms

    // 2. SetRfFrequency
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_freq, 4, 100);
    My_spi_nss_close();
    HAL_Delay(20);

    // 3. SetTxParams
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_txp, 3, 100);
    My_spi_nss_close();
    HAL_Delay(20);

    // 4. SetBufferBaseAddress
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_base, 3, 100);
    My_spi_nss_close();
    HAL_Delay(20);

    // 5. SetModulationParams
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_mod, 4, 100);
    My_spi_nss_close();
    HAL_Delay(20);

    // 6. SetPacketParams
    My_spi_nss_en();
	HAL_Delay(5);
    HAL_SPI_Transmit(&hspi1, cmd_pkt, 8, 100);
    My_spi_nss_close();
    HAL_Delay(20);
    //同步字
uint8_t sync_cmd1[] = {0x18, 0x09, 0x44, 0x12}; 
uint8_t sync_cmd2[] = {0x18, 0x09, 0x45, 0x44};

My_E28_send(sync_cmd1, 4);
My_E28_send(sync_cmd2, 4);

    My_E28_Enable_Internal_IRQ();

    // // //用于调试看有没有真的通讯成功
    // uint8_t read_cmd[5] = {0x19, 0x01, 0x50, 0x00, 0x00}; // 指令(1) + 地址(2) + 虚拟字节(1) + 额外一个字节放数据
    // uint8_t read_res[5] = {0};
    // uint8_t getstatus_cmd[]={0xc0};
    // My_spi_nss_en();
    // // 使用同时收发函数
    // HAL_SPI_TransmitReceive(&hspi1, read_cmd, read_res, 5, 100);
    // My_spi_nss_close();

    // // 根据 SX1280 时序：
    // // read_res[0] 是 Status
    // // read_res[1] 是 Status
    // // read_res[2] 是 Status
    // // read_res[3] 才是寄存器 0x0150 的真正数据 (由于指令要求1个dummy)

    // //调试0xc0用于getstattus，返回一个字节
    //  My_spi_nss_en();
    // HAL_SPI_TransmitReceive(&hspi1, getstatus_cmd, read_res, 5, 100);
    // My_spi_nss_close();
    // HAL_Delay(20);
}

void My_E28_2G4M20S_send(uint8_t *buf,uint8_t buf_size){
    
}

void My_E28_2G4M20S_receive(){

}

void My_spi_nss_en()//使能nss，他平常的时候是高电平，开始通信的时候是低电平
{
    HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port,SPI1_NSS1_Pin,0);
}

void My_spi_nss_close()//关闭nss
{
    HAL_GPIO_WritePin(SPI1_NSS1_GPIO_Port,SPI1_NSS1_Pin,1);
}





// void My_E28_change_to_transmitmode(){//1,0,0

//     issendcmd_flag=0;
//     HAL_GPIO_WritePin(M1_GPIO_Port,M1_Pin,0);
//     HAL_GPIO_WritePin(M0_GPIO_Port,M0_Pin,0);

// }

// void My_E28_change_to_writecmd()//通信波特率要设置为9600
// {
//     issendcmd_flag=1;
//     HAL_GPIO_WritePin(M1_GPIO_Port,M1_Pin,1);
//     HAL_GPIO_WritePin(M0_GPIO_Port,M0_Pin,1);
// }


// void My_E28_init_usartmode()
// { // 准备指令数组
    
//     My_E28_change_to_writecmd();//uart透传模块转换模式
// 	HAL_Delay(100); // 必须给一点时间让电平稳定
//     uint8_t cmd_type[] = {0x8a, 0x01,0x01};     //packettype->loramode                          
//     uint8_t cmd_freq[] = {0x86,0x03, 0xb8, 0x9d, 0x89};                   
//     uint8_t cmd_txp[]  = {0x8e,0x02, 0x1f, 0xe0};     //settxparam           
//     uint8_t cmd_base[] = {0x8f, 0x02,0x80, 0x00};     //设置缓冲区地址           
//     uint8_t cmd_mod[]  = {0x8b, 0x03,0x70, 0x0a, 0x01};                   
//     uint8_t cmd_pkt[]  = {0x8c,0x07, 0x0c, 0x00, 0x80, 0x20, 0x40, 0x00, 0x00};  
//     HAL_UART_Transmit(&huart1,cmd_type,3,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);

//     HAL_UART_Transmit(&huart1,cmd_freq,5,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);

//     HAL_UART_Transmit(&huart1,cmd_txp,4,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);

//     HAL_UART_Transmit(&huart1,cmd_base,4,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);

//     HAL_UART_Transmit(&huart1,cmd_mod,5,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);

//     HAL_UART_Transmit(&huart1,cmd_pkt,9,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);
//     //用于调试看有没有真的通讯成功

// uint8_t recbuf[6] = {0};

// uint8_t b=0;

// My_E28_change_to_writecmd();
// HAL_Delay(100);

// uint8_t cmd[3] = {0xC3, 0xC3, 0xC3};
// HAL_UART_Transmit(&huart1, cmd, 3, 100);

// HAL_Delay(30); // 给模块反应时间

// HAL_UART_Receive(&huart1,recbuf,6,0);

// }

// void My_E28_receive_mode(){//setrx
//     My_E28_change_to_writecmd();
//     uint8_t rx_cmd[]={0x82,0x03,0x03,0x00,0xFA};
//     HAL_UART_Transmit(&huart1,rx_cmd,5,100);
//     while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == 0);
//     My_E28_change_to_transmitmode();

// }

// void My_E28_standby(){
//     uint8_t buf[2];
//     buf[0]=0x80;
//     buf[1]=0x00;
//     HAL_UART_Transmit(&huart1,buf,2,100);
// }


