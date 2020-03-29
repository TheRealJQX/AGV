#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>



/**
* L298N���ƣ�
* ��ǰ����ת��ENA=1;IN1=0;IN2=1;
* �Һ�����ת��ENB=1;IN3=1;IN4=0;
* ɲ���� ENA=1;IN1=0;IN2=0;��ENA=1;IN1=1;IN2=1;
*/

/**
*	�������1��
* stm32ͨ��NodeMcu�������ݣ�
* 0��������1��ҵ��
*/



//USART1���ڣ����ţ�PB6(��PA9)-TX1��PB7(PA10)-RX1
//

// ������ǰ��
#define  ENA   PAout(7)
#define  IN1   PAout(6)
#define  IN2   PAout(5)

// �����Һ���
#define  ENB   PAout(2)
#define  IN3   PAout(3)
#define  IN4   PAout(4)

//void INT_IO_Init(void);
void L298N_Roll(void);
//#include "usart.h"
// #include "DHT11.h"
#include "delay.h"
#include "sys.h"
// #include "lcd.h"

// #define year_days(a) (((a%100!=0)&&(a%4==0))||(a%400==0))   
// #define days_in_year(i) (year_days(i) ? 366 : 365)  
 
// unsigned long days_in_month[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};


static int flag = 0;
char multip_scaner_cmd[] = "AT+SCAN=0\r\n";	//ȫ��ɨ��
char single_scaner_cmd[] = "AT+SCAN=1\r\n";	//����ɨ��

void  USART1_Puts(u8 * str);
void  GPIO_Configuration(void) ;
void 	RCC_Configuration(void);
void  uart_init(u32 bound);
//void  ADC_Configuration(void) ;
//void  RTC_Configuration(void);
//void  RTCClock_Init(void);
void  NVIC_Configuration(void);
void  textLED();
//u16 Get_val(u8 ch);
//int tim_bz=0;
//u32 ADC_ConvertedValue;
static uint8_t trace_signal=0;
static uint8_t start_signal=0;



void L298N_Straight(void)
{
	 ENA=1;
   IN1=1;
	 IN2=0;
   
	 ENB=1;
	 IN3=0;
	 IN4=1;
	
	 delay_ms(100);
}

void L298N_Stop(void)
{
	 ENA=0;
   IN1=1;
	 IN2=1;
   
	 ENB=0;
	 IN3=1;
	 IN4=1;
	
	 delay_ms(100);
}

u8  USART_BUF[10];
u16 USART_RX_STA;
u16 USART_STA=0;       //����״̬���
static u8 MISSION_POINT;     //������ʼ�����־
static u8 PART_AREA;      //Ŀ��ͣ�������־
static u8 PART_POINT;			//Ŀ��ͣ����
u8 Motor;         //̧����־
u8 situation[5];  //��λ��Ϣ���� 

/*��0λ����ҵ��;1-3����ҵ�ߣ�0Ϊ����״̬ͣפ
 �� ��1λ��̧����ʶ;1̧2��0��
  ��2λ������״̬��ʶ��0�û�����δ���أ�1�û������Ѵ���
	��3λ��Ŀ�공λ��a-h

	
*/



int main(void)
{

  // �����ź�
	//uint8_t trace_signal_neg=0;// �����ź��ж���
  SystemInit();
	RCC_Configuration();
	GPIO_Configuration();
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);

	//NVIC_Configuration();
	uart_init(115200);
	 
	delay_init();
	
	while(1){


	if(!start_signal)
		//����״̬	
		{
	 	L298N_Stop();
	
		}
			else{	
 // ���µ����ź�
  trace_signal = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
	
 	
		
		if(trace_signal==1){
				//��⵽�ߵ�ƽ��ƫ����ǳɫ���� 
			L298N_Stop();
			}
			else if(trace_signal==0){
				//��⵽�͵�ƽ��ѭ����ɫ����
				L298N_Straight();
				
			}
			else{
				//�쳣�����̨����״̬
				delay_ms(1000);
				L298N_Stop();
			}
			
			delay_ms(100);
	}
}
	
	
}


void  GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ������
	
		//USART2_RX:PA10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//USART2_TX:PA9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����Ƶ��50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��
	

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;   //���ùܽ�	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//���������
	GPIO_Init(GPIOC,&GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   //���ùܽ�	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//���������
	GPIO_Init(GPIOA,&GPIO_InitStructure);//��ʼ��

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6; //���ùܽ�PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //IO������Ϊ���������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //���ùܽ�8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //IO������Ϊ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����Ƶ��50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   //���ùܽ�	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//���������
	GPIO_Init(GPIOA,&GPIO_InitStructure);//��ʼ��


  // RX2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3; //���ùܽ�PA9/USART1_TX
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //IO������Ϊ���������
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //��ʼ��
	
	//TX2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //���ùܽ�8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //IO������Ϊ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����Ƶ��50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��
	
	//�����������ܽ�PB12��PB13��PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //���ùܽ�B1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //IO������Ϊ��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����Ƶ��50MHz
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��
	
	
	

}

void uart_init(u32 bound){
  //GPIO�˿�����
	USART_InitTypeDef USART_InitStructure;
	 

   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	NVIC_Configuration();
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	USART_ClearITPendingBit(USART1,USART_FLAG_ORE);
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
  USART_ClearFlag(USART1, USART_FLAG_ORE);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
 // USART_ITConfig(USART3, USART_IT_TXE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
  //USART_ClearFlag(USART3,USART_FLAG_TC);

}



void RCC_Configuration(void){
	SystemInit();
	
	// PA�ܽſ��Ƴ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��GPIOA��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//ʹ��GPIOA��ʱ��

	// PB�ܽſ��ƴ�����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��GPIOB��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 ,ENABLE);	//ʹ��USART1

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1��ʱ�ӣ� USART1�ҽӵ�APB2�ϡ�����USART2-5�ҽӵ�APB1��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 
}


void  ADC_Configuration(void) {
	ADC_InitTypeDef  ADC_InitStructure;
	
	ADC_TempSensorVrefintCmd(ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode  = ADC_Mode_Independent;//ѡ��ADC�����ڶ���ģʽ����˫ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode  = ENABLE;  //Ȩ������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode  = ENABLE;//ѡ��ADC�������������ǵ���ģʽ��

	ADC_InitStructure.ADC_ExternalTrigConv  = ADC_ExternalTrigConv_None;//����ʹ���ⲿ��������������ͨ����ADC
	ADC_InitStructure.ADC_DataAlign  = ADC_DataAlign_Right;//������������뻹���Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel  = 1;//���ý��й���ת����ADCͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1
	
	ADC_ResetCalibration(ADC1);  // ��λADC1У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));  //�ȴ�У׼��λ����

	ADC_StartCalibration(ADC1);//��ʼУ׼
	while(ADC_GetCalibrationStatus(ADC1)); //�ȴ�У׼����
	
	//ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


//Usart1 NVIC ����
void NVIC_Configuration(void)
{
	 
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void textLED(void){
   			if (flag) {flag=0;GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
    // USART_SendData(USART1,USART_ReceiveData(USART1)); 

}else {flag=1;GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);}
		delay_ms(500);
}	


void USART1_IRQHandler(void)
{    
	    
	//	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);

      u8 res = 0;
	  //  u8 *buf = USART_BUF;
	  //  u8 area_flag; //�ڸô����ж��У����յ��������ĸ��
	// �򽫸�������������Ŀ����Ϣ��
	// ��д��ĸ��������飬
	// Сд��ĸ�������λ�š�
	
		//USART_SendData(USART1, USART_ReceiveData(USART1));

   //  USART_SendData(USART3,3);
     if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)//ע�⣡����ʹ��if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)���ж�  
    {  
			  USART_ClearFlag(USART1, USART_FLAG_ORE);
			  USART_ClearITPendingBit(USART1,USART_FLAG_ORE); 
        USART_ReceiveData(USART1);			
       // USART_SendData(USART3, USART_ReceiveData(USART3)); 
			  
    }  
		
		
   //���ڴ�����һ��ͨѶ�����յ����ݲ�����  
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
    {   
        
			//ucRS485Buff[ucRcePtr++] = USART_ReceiveData(USART1);  
       textLED();
       USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			 USART_ClearFlag(USART1, USART_IT_RXNE);
			 res = USART_ReceiveData(USART1);
	
					
			 if(res==0x00) {
				 L298N_Stop();
				 start_signal = 0;
			 }
			 //���ֿ�ͷ����������ʼ����Ϣ
			 else if (res>=0x01&&res<=0x03){
		     start_signal = 1;
				 PART_POINT = res;
				
			 
			 }
			 else if(res>=0x41&&res<=0x5a){
				  start_signal = 1;
					PART_AREA = res;
			 }
			 
			 
			  USART_SendData(USART1, res);
    }  
		  USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	    USART_ClearITPendingBit(USART1,USART_FLAG_ORE);
		  USART_ClearFlag(USART1, USART_IT_RXNE);
		  USART_ClearFlag(USART1, USART_FLAG_ORE);
		 // USART_RX_STA = 0;
     // USART1_Puts(buf);
}

void USART1_Puts(u8 * str)
{
    while(*str)
    {
        USART1->DR= *str++;
        while((USART1->SR&0X40)==0);//�ȴ��������  
    }
}
/*
void RTC_Configuration(void)

{
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP,ENABLE);
   //ʹ�ܵ�Դ�ͺ���
    PWR_BackupAccessCmd(ENABLE);
    // �������BKP����
    BKP_DeInit();
    //BKP��λ
    RCC_LSEConfig(RCC_LSE_ON);
    //�����ⲿ���پ���  32.768k
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    //�ȴ��ⲿ���׾����Ƿ��Ѿ��ȶ� 
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    //ѡ��RTC�ⲿʱ��Ϊ���׾���
    RCC_RTCCLKCmd(ENABLE);
    //ʹ��RTCʱ��
    RTC_WaitForSynchro();
    //�ȴ�RTC�Ĵ���ͬ��            
    RTC_WaitForLastTask();
    //�ȴ�дRTC�Ĵ������
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    //ʹ��RTC���ж�       
    RTC_WaitForLastTask();
		RTC_SetPrescaler(32767);
		//����Ԥ��Ƶ
		RTC_WaitForLastTask();
}

void RTCClock_Init(void){
	if(BKP_ReadBackupRegister(BKP_DR1)!=0xA5A5){
		//��һ������  ��ʼ������
		//RTC��ʼ��
		RTC_Configuration();
		//�ȴ�дRTC�Ĵ������
		RTC_WaitForLastTask();
    //����ʱ���ֵ
		RTC_SetCounter(0x001B82BC);
		RTC_WaitForLastTask();
		//д�����ñ�־
		BKP_WriteBackupRegister(BKP_DR1,0xA5A5);
	}
	else{
		RTC_WaitForSynchro();
    //�ȴ�RTC�Ĵ���ͬ��            
    RTC_WaitForLastTask();
    //�ȴ�дRTC�Ĵ������
		 RTC_ITConfig(RTC_IT_SEC, ENABLE);
    //ʹ��RTC���ж�   
		RTC_WaitForLastTask();
		
	}
	
	RCC_ClearFlag();    //�����λ��־
}
*/
/*void RTC_IRQHandler(void) {
	if(RTC_GetITStatus(RTC_IT_SEC)!= RESET)//��ȡ�жϱ�־
	{
		RTC_ClearITPendingBit(RTC_IT_SEC);//����жϱ�־
	//	tim_bz=1;//���жϱ�־
	}
}*/
/*
u16 Get_val(u8 ch)
{
    u16 DataValue; //����Ӣ��ע�͡�������    
		
			ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
    
   
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
   

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

		
		DataValue = ADC_GetConversionValue(ADC1); 
		return DataValue; 
} 

*/



