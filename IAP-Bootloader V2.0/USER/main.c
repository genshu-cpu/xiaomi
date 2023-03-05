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
 * Filename   ��main.c
 * Author     : PuSheng 
 * Version    : 2.0
 * Date       : 2019.12.31

 * Discription : ���ݴ���Э��͹̼������ܹ�
 
 v1.0:�������Э��͹̼������������

*/

#define  PRODUCT_APP_PASSWORD (0xFee1Dead)  //��ƷӦ����Կ
#define  PASSWORD_ADDR        (0x08005000)  //��ŵ�ַ20kb
u32  ProductAppPassword  =     PRODUCT_APP_PASSWORD;  //��ƷӦ����Կ

#define  APP_ADDR  (0x08010000)  //�̼���ַ 64KB֮��

#define  TMP_BUF_SIZE  2*1024 //һ����������С  ��Ҫ��2��ָ��ֵ
#define  TMP_BUF_NUM   5      //����������

u8 FirmwareBuf[TMP_BUF_NUM][TMP_BUF_SIZE]={0};//�������ڴ�ռ�

/*��д��һ�������� ��Ҫ�������º��� ָ���һ*/
#define WRITE_BUF_DONE()  tAI.WriteIndex = ( (++tAI.WriteIndex >= TMP_BUF_NUM ) ? 0:tAI.WriteIndex);\
                          if(tAI.WriteIndex == 0 )tAI.IsCircularListOverflow = true;//������
                              
#define READ_BUF_DONE()   tAI.ReadIndex = ( (++tAI.ReadIndex >= TMP_BUF_NUM ) ? 0:tAI.ReadIndex);\
                          if(tAI.ReadIndex == 0 )tAI.IsCircularListOverflow = false;//�������
typedef struct
{
    
    
    u32  AppSize;//�̼���С Byte
    u8   *TmpBuf[TMP_BUF_NUM];//����buf
    u32  TmpBufSize;//һ����������С
    bool IsCircularListOverflow;//�Ƿ�дָ�볬��һ��ѭ��
    u8   ReadIndex;//������bufָ��
    u8   WriteIndex;//д�뻺��bufָ��
    
}AppInfor_t;

AppInfor_t tAI;

/*
* Function Name  : vAppInforInit
* Description    : ��ȫ������tAI��ʼ��
* Input          : t AppInfor_t����
* Output         : None 
* Return         : None
*/
void vAppInforInit(AppInfor_t *t)
{
    u8 i=0;
    for(i=0;i<TMP_BUF_NUM;i++)
    {
        t->TmpBuf[i]=FirmwareBuf[i];//������ָ��
    }
    t->AppSize=0;//�̼���С����
    t->TmpBufSize=TMP_BUF_SIZE;//��ֵ��������С
    t->IsCircularListOverflow=false;//�������û�����
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
    if(*(u32 *)PASSWORD_ADDR == PRODUCT_APP_PASSWORD)//�Ѿ��й̼���
    {
        iap_load_app(APP_ADDR);//��ת���̼�ִ��
    }
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	  
	LED_Init();		  					//��ʼ��LED
    vUsart1DmaInitForTest();
	vUsart2DmaInitForTest();
    //printf("OK\r\n");

    vAppInforInit(&tAI);
	//������ʼ����
    xTaskCreate(vLed_Task,"vLed_Task",40,0,1,0);  
    xTaskCreate(vUsartDataHandleTask,"vUsartDataHandleTask",40,0,1,0);
    xTaskCreate(vFrameDataHandlerTask,"vFrameDataHandlerTask",120,0,1,0);
    xTaskCreate(vFirmwareWriteTask,"vFirmwareWriteTask",120,0,1,0);   

    vTaskStartScheduler();          //�����������
}



/*
* Function Name  : vLed_Task
* Description    : LED��˸������
* Input          : ���� 
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
* Description    : �������ݽ����뷢��������
* Input          : ���� 
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
* Description    : Э��֡���ݴ���������
* Input          : ���� 
* Output         : None 
* Return         : None
*/
void vFrameDataHandlerTask(void *p)
{
    u8 length=0;
    while(1)
    {
        /*���ڽ��յ����� ����֡�����Ѿ�ȫ���������*/
        if(ucUSART1_ReceiveMessageNumber && FR_Rx.IsFrameNotProcess == false)
        {
            length=ucUSART1ReceiveBufferLength[ucUSART1ReadBufferIndex];//��ȡ�������ݵĳ���
            eFrame_Analy(ucUSART1ReceiveBuffer[ucUSART1ReadBufferIndex],length,&FR_Rx);//��������
            USART1ReceiveDataReadDone(  );//�������ݶ�ȡ���
            
            /*�ɹ����յ��Ĳ��ǻ�Ӧ֡*/
            if( FR_Rx.tFrame.FrameFunction != ffRespond  )
            {
                /*���û�Ӧ֡*/
                ucCode_RespondFrame(&FR_Tx,&FR_Rx,ucUSART1TrainsmitBuffer[ucWtiteDataToUSART1TransmitGrooveIndex]);
                /*���ͻ�Ӧ֡�����ڷ��ͻ�����*/
                ucUSART1TrainsmitLength[ucWtiteDataToUSART1TransmitGrooveIndex]=5+FR_Tx.tFrame.DataLength;
                WriteDataToUSART1TraismitBufferDone();
            }              
        }
        
        /*֡���ݴ���*/
        if(FR_Rx.IsFrameNotProcess)
        {
            switch(FR_Rx.tFrame.FrameFunction)
            {
                case ffRespond: break;//���յ����ǻ�Ӧ֡��������
                case ffCommand: //���յ���������֡
                    vCommandFrameHandle(&FR_Rx);
                    break;
                
                case ffData: //���յ���������֡
                    vDataFrameHandle(&FR_Rx);
                    break;
                case ffIdle: break;//���յ����ǿ���֡��������
                case ffMax: break;
            }
            FR_Rx.IsFrameNotProcess=false;//��������Ѿ�����
        }
        
        vTaskDelay(5);
    }
}

/*
* Function Name  : vCommandFrameHandle
* Description    : ����֡������
* Input          : Rx_FrameRecord ����֡�ṹ�� 
* Output         : None 
* Return         : None
*/
fCommand_t LastCommand;//�ϴν��յ�������
void vCommandFrameHandle(Rx_FrameRecord *R)
{
    if( R->tFrame.DataLength==1 )//�����ֽ�ֻ��һ�ֽ�
    {
        switch(R->tFrame.FrameData[0])//�鿴����
        {
            case UpdateFirmwareStart: //�ǿ�ʼ���¹̼�����
                ProductAppPassword=0;
                STMFLASH_Write(PASSWORD_ADDR,(u16*)&ProductAppPassword,2);//������Ʒ��Կ
                LastCommand=UpdateFirmwareStart;//
                tAI.AppSize=0;
            
                break;
            
            case UpdateFirmwareDone: //�ǽ������¹̼�����
                LastCommand=UpdateFirmwareDone;
                break;
        }
        
    }
}
/*
* Function Name  : vDataFrameHandle
* Description    : ����֡������
* Input          : Rx_FrameRecord ����֡�ṹ�� 
* Output         : None 
* Return         : None
*/
void vDataFrameHandle(Rx_FrameRecord *R)
{
    static u32 Count=0;//����������
    u32 SurplusCount=0;//ʣ�����ݼ��� 
    u32 CanWriteCount=0;// ����д������ݸ��� 
    switch(LastCommand)
    {
        case UpdateFirmwareStart://����Ϊ��ʼ���¹̼�
            
            if( (Count + R->tFrame.DataLength) > TMP_BUF_SIZE)//���ݳ���������
            {
                CanWriteCount=TMP_BUF_SIZE-Count;//������������д������ݸ���
                tAI.AppSize+=CanWriteCount;
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),R->DataBuf,CanWriteCount);//����һ��������
                WRITE_BUF_DONE();//���һ��������д�����
                
                if(tAI.IsCircularListOverflow == true)//�Ƿ����
                {
                    while( tAI.WriteIndex == tAI.ReadIndex ) //д��ָ���Ƿ�Ҫ������ȡָ��
                    {
                        vTaskDelay(10);//��ֹ���ݼ�̤ �ȴ��������Ķ�ȡ���
                    }
                }
                SurplusCount=R->tFrame.DataLength-CanWriteCount;//��ǻ��ж�������û��д��  
                Count=0;//�����һ������Count�ļ���
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),&(R->DataBuf[CanWriteCount]),SurplusCount);//��ʣ��֡����д�뻺����
                Count+=SurplusCount;
                tAI.AppSize+=SurplusCount; //�ı�̼���С
            }else
            {
                memcpy(&(tAI.TmpBuf[tAI.WriteIndex][Count]),R->DataBuf,R->tFrame.DataLength);//��֡����д�뻺����
                Count += R->tFrame.DataLength;//���ϱ�֡���ݳ���
                tAI.AppSize+=R->tFrame.DataLength; //�ı�̼���С
            }
            
            break;
        case UpdateFirmwareDone://����Ϊ�̼�������� ����ִ��
            break;
            
    }

}

/*
* Function Name  : vFirmwareWriteTask
* Description    : �̼�д�뵽FLASH����
* Input          : ����
* Output         : None 
* Return         : None
*/
void  vFirmwareWriteTask(void *p)
{
    static u32 WriteFirmwareCount=0;
    while(1)
    {
        if(tAI.IsCircularListOverflow == false)//��ָ���дָ��û�����һ��ѭ��
        {
            while(tAI.ReadIndex == tAI.WriteIndex)//����Ƿ�д��һ�����ݻ�����
            {
                if(LastCommand == UpdateFirmwareDone)//���ܵ��̼������������
                {
                    /*д��ʣ������*/
                    iap_write_appbin(APP_ADDR+WriteFirmwareCount,tAI.TmpBuf[tAI.ReadIndex],tAI.AppSize-WriteFirmwareCount);
                    WriteFirmwareCount+=(tAI.AppSize-WriteFirmwareCount);
                    
                    /*��ת��APP����*/
//                    INTX_DISABLE();
//                    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//�رմ��ڽ����ж�
//                    USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);//�رմ��ڿ����ж�
                    
                    ProductAppPassword=PRODUCT_APP_PASSWORD;
                    STMFLASH_Write(PASSWORD_ADDR,(u16*)&ProductAppPassword,2);//д���Ʒ��Կ
                    iap_load_app(APP_ADDR);//��ת���̼�ִ��
//                    while(1){};
                }
                vTaskDelay(15);//�ȴ�����д�����
            }   
        }
        /*��һ������������д��FLASH*/
        iap_write_appbin(APP_ADDR+WriteFirmwareCount,tAI.TmpBuf[tAI.ReadIndex],TMP_BUF_SIZE);
        WriteFirmwareCount+=TMP_BUF_SIZE;
        
        READ_BUF_DONE();//��ȡ��һ��������
        vTaskDelay(5);
    }
}

