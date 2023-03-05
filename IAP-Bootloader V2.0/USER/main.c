#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"
#include "string.h"
#include "iap.h"
#include "stmflash.h"

/*
 * Filename   ：main.c
 * Author     : PuSheng 
 * Version    : 2.0
 * Date       : 2019.12.31

 * Discription : 数据传输协议和固件升级架构
 
 v1.0:完成数据协议和固件升级基本框架

*/

#define  PRODUCT_APP_PASSWORD (0xFee1Dead)  //产品应用秘钥
#define  PASSWORD_ADDR        (0x08005000)  //存放地址20kb
u32  ProductAppPassword  =     PRODUCT_APP_PASSWORD;  //产品应用秘钥

#define  APP_ADDR  (0x08010000)  //固件地址 64KB之后

#define  TMP_BUF_SIZE  2*1024 //一个缓冲区大小  需要是2的指数值
#define  TMP_BUF_NUM   5      //缓冲区个数

u8 FirmwareBuf[TMP_BUF_NUM][TMP_BUF_SIZE]={0};//缓冲区内存空间

/*读写完一个缓存区 需要调用以下函数 指针加一*/
#define WRITE_BUF_DONE()  tAI.WriteIndex = ( (++tAI.WriteIndex >= TMP_BUF_NUM ) ? 0:tAI.WriteIndex);\
                          if(tAI.WriteIndex == 0 )tAI.IsCircularListOverflow = true;//标记溢出
                              
#define READ_BUF_DONE()   tAI.ReadIndex = ( (++tAI.ReadIndex >= TMP_BUF_NUM ) ? 0:tAI.ReadIndex);\
                          if(tAI.ReadIndex == 0 )tAI.IsCircularListOverflow = false;//重置溢出
typedef struct
{
    
    
    u32  AppSize;//固件大小 Byte
    u8   *TmpBuf[TMP_BUF_NUM];//缓存buf
    u32  TmpBufSize;//一个缓存区大小
    bool IsCircularListOverflow;//是否写指针超过一次循环
    u8   ReadIndex;//读缓存buf指针
    u8   WriteIndex;//写入缓存buf指针
    
}AppInfor_t;

AppInfor_t tAI;

/*
* Function Name  : vAppInforInit
* Description    : 将全局数据tAI初始化
* Input          : t AppInfor_t类型
* Output         : None 
* Return         : None
*/
void vAppInforInit(AppInfor_t *t)
{
    u8 i=0;
    for(i=0;i<TMP_BUF_NUM;i++)
    {
        t->TmpBuf[i]=FirmwareBuf[i];//缓冲区指定
    }
    t->AppSize=0;//固件大小清零
    t->TmpBufSize=TMP_BUF_SIZE;//赋值缓冲区大小
    t->IsCircularListOverflow=false;//标记数据没有溢出
    t->ReadIndex=0;
    t->WriteIndex=0;
}

void  vLed_Task(void *pvParameters);
void  vUsartDataHandleTask(void *p);
void  vFrameDataHandlerTask(void *p);
void  vFirmwareWriteTask(void *p);

void  vCommandFrameHandle(Rx_FrameRecord *R);
void  vDataFrameHandle(Rx_FrameRecord *R);

int main(void)
{
    if(*(u32 *)PASSWORD_ADDR == PRODUCT_APP_PASSWORD)//已经有固件了
    {
        iap_load_app(APP_ADDR);//跳转到固件执行
    }
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	delay_init();	    				//延时函数初始化	  
	LED_Init();		  					//初始化LED
    vUsart1DmaInitForTest();
	vUsart2DmaInitForTest();
    //printf("OK\r\n");

    vAppInforInit(&tAI);
	//创建开始任务
    xTaskCreate(vLed_Task,"vLed_Task",40,0,1,0);  
    xTaskCreate(vUsartDataHandleTask,"vUsartDataHandleTask",40,0,1,0);
    xTaskCreate(vFrameDataHandlerTask,"vFrameDataHandlerTask",120,0,1,0);
    xTaskCreate(vFirmwareWriteTask,"vFirmwareWriteTask",120,0,1,0);   

    vTaskStartScheduler();          //开启任务调度
}



/*
* Function Name  : vLed_Task
* Description    : LED闪烁任务函数
* Input          : 参数 
* Output         : None 
* Return         : None
*/
void vLed_Task(void *pvParameters)
{
    while(1)
    {
        LED0=~LED0;
        vTaskDelay(500);
    }
}   

/*
* Function Name  : vUsartDataHandleTask
* Description    : 串口数据接收与发送任务函数
* Input          : 参数 
* Output         : None 
* Return         : None
*/
void vUsartDataHandleTask(void *p)
{
    while(1)
    {
        vUsart1Interactive();
        vUsart2Interactive();
        
        vTaskDelay(5);
    }
}

/*
* Function Name  : vFrameDataHandlerTask
* Description    : 协议帧数据处理任务函数
* Input          : 参数 
* Output         : None 
* Return         : None
*/
void vFrameDataHandlerTask(void *p)
{
    u8 length=0;
    while(1)
    {
        /*串口接收到数据 并且帧数据已经全部处理完毕*/
        if(ucUSART1_ReceiveMessageNumber && FR_Rx.IsFrameNotProcess == false)
        {
            length=ucUSART1ReceiveBufferLength[ucUSART1ReadBufferIndex];//获取接收数据的长度
            eFrame_Analy(ucUSART1ReceiveBuffer[ucUSART1ReadBufferIndex],length,&FR_Rx);//解析数据
            USART1ReceiveDataReadDone(  );//串口数据读取完毕
            
            /*成功接收到的不是回应帧*/
            if( FR_Rx.tFrame.FrameFunction != ffRespond  )
            {
                /*配置回应帧*/
                ucCode_RespondFrame(&FR_Tx,&FR_Rx,ucUSART1TrainsmitBuffer[ucWtiteDataToUSART1TransmitGrooveIndex]);
                /*发送回应帧到串口发送缓冲区*/
                ucUSART1TrainsmitLength[ucWtiteDataToUSART1TransmitGrooveIndex]=5+FR_Tx.tFrame.DataLength;
                WriteDataToUSART1TraismitBufferDone();
            }              
        }
        
        /*帧数据处理*/
        if(FR_Rx.IsFrameNotProcess)
        {
            switch(FR_Rx.tFrame.FrameFunction)
            {
                case ffRespond: break;//接收到的是回应帧，不处理
                case ffCommand: //接收到的是命令帧
                    vCommandFrameHandle(&FR_Rx);
                    break;
                
                case ffData: //接收到的是数据帧
                    vDataFrameHandle(&FR_Rx);
                    break;
                case ffIdle: break;//接收到的是空闲帧，不处理
                case ffMax: break;
            }
            FR_Rx.IsFrameNotProcess=false;//标记数据已经处理
        }
        
        vTaskDelay(5);
    }
}

/*
* Function Name  : vCommandFrameHandle
* Description    : 命令帧处理函数
* Input          : Rx_FrameRecord 接收帧结构体 
* Output         : None 
* Return         : None
*/
fCommand_t LastCommand;//上次接收到的命令
void vCommandFrameHandle(Rx_FrameRecord *R)
{
    if( R->tFrame.DataLength==1 )//命令字节只有一字节
    {
        switch(R->tFrame.FrameData[0])//查看命令
        {
            case UpdateFirmwareStart: //是开始更新固件命令
                ProductAppPassword=0;
                STMFLASH_Write(PASSWORD_ADDR,(u16*)&ProductAppPassword,2);//擦除产品秘钥
                LastCommand=UpdateFirmwareStart;//
                tAI.AppSize=0;
            
                break;
            
            case UpdateFirmwareDone: //是结束更新固件命令
                LastCommand=UpdateFirmwareDone;
                break;
        }
        
    }
}
/*
* Function Name  : vDataFrameHandle
* Description    : 数据帧处理函数
* Input          : Rx_FrameRecord 接收帧结构体 
* Output         : None 
* Return         : None
*/
void vDataFrameHandle(Rx_FrameRecord *R)
{
    static u32 Count=0;//缓冲区计数
    u32 SurplusCount=0;//剩余数据计数 
    u32 CanWriteCount=0;// 可以写入的数据个数 
    switch(LastCommand)
    {
        case UpdateFirmwareStart://命令为开始更新固件
            
            if( (Count + R->tFrame.DataLength) > TMP_BUF_SIZE)//数据超过缓冲区
            {
                CanWriteCount=TMP_BUF_SIZE-Count;//缓冲区还可以写入的数据个数
                tAI.AppSize+=CanWriteCount;
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),R->DataBuf,CanWriteCount);//填满一个缓冲区
                WRITE_BUF_DONE();//标记一个缓冲区写入完成
                
                if(tAI.IsCircularListOverflow == true)//是否溢出
                {
                    while( tAI.WriteIndex == tAI.ReadIndex ) //写入指针是否要超过读取指针
                    {
                        vTaskDelay(10);//防止数据践踏 等待缓冲区的读取完毕
                    }
                }
                SurplusCount=R->tFrame.DataLength-CanWriteCount;//标记还有多少数据没有写入  
                Count=0;//清空上一缓存区Count的计数
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),&(R->DataBuf[CanWriteCount]),SurplusCount);//将剩余帧数据写入缓冲区
                Count+=SurplusCount;
                tAI.AppSize+=SurplusCount; //改变固件大小
            }else
            {
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),R->DataBuf,R->tFrame.DataLength);//将帧数据写入缓冲区
                Count += R->tFrame.DataLength;//加上本帧数据长度
                tAI.AppSize+=R->tFrame.DataLength; //改变固件大小
            }
            
            break;
        case UpdateFirmwareDone://命令为固件更新完毕 不会执行
            break;
            
    }

}

/*
* Function Name  : vFirmwareWriteTask
* Description    : 固件写入到FLASH任务
* Input          : 参数
* Output         : None 
* Return         : None
*/
void  vFirmwareWriteTask(void *p)
{
    static u32 WriteFirmwareCount=0;
    while(1)
    {
        if(tAI.IsCircularListOverflow == false)//读指针和写指针没有相隔一个循环
        {
            while(tAI.ReadIndex == tAI.WriteIndex)//检测是否写完一个数据缓冲区
            {
                if(LastCommand == UpdateFirmwareDone)//接受到固件更新完毕命令
                {
                    /*写入剩余数据*/
                    iap_write_appbin(APP_ADDR+WriteFirmwareCount,tAI.TmpBuf[tAI.ReadIndex],tAI.AppSize-WriteFirmwareCount);
                    WriteFirmwareCount+=(tAI.AppSize-WriteFirmwareCount);
                    
                    /*跳转到APP代码*/
//                    INTX_DISABLE();
//                    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//关闭串口接受中断
//                    USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);//关闭串口空闲中断
                    
                    ProductAppPassword=PRODUCT_APP_PASSWORD;
                    STMFLASH_Write(PASSWORD_ADDR,(u16*)&ProductAppPassword,2);//写入产品秘钥
                    iap_load_app(APP_ADDR);//跳转到固件执行
//                    while(1){};
                }
                vTaskDelay(15);//等待数据写入完毕
            }   
        }
        /*将一个缓冲区数据写入FLASH*/
        iap_write_appbin(APP_ADDR+WriteFirmwareCount,tAI.TmpBuf[tAI.ReadIndex],TMP_BUF_SIZE);
        WriteFirmwareCount+=TMP_BUF_SIZE;
        
        READ_BUF_DONE();//读取完一个缓冲区
        vTaskDelay(5);
    }
}

