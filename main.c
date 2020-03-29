#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>



/**
* L298N控制：
* 右前轮正转：ENA=1;IN1=0;IN2=1;
* 右后轮正转：ENB=1;IN3=1;IN4=0;
* 刹车： ENA=1;IN1=0;IN2=0;或ENA=1;IN1=1;IN2=1;
*/

/**
*	命令设计1：
* stm32通过NodeMcu接收数据，
* 0：待机；1作业；
*/



//USART1串口，引脚：PB6(或PA9)-TX1；PB7(PA10)-RX1
//

// 控制右前轮
#define  ENA   PAout(7)
#define  IN1   PAout(6)
#define  IN2   PAout(5)

// 控制右后轮
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
char multip_scaner_cmd[] = "AT+SCAN=0\r\n";	//全速扫码
char single_scaner_cmd[] = "AT+SCAN=1\r\n";	//单次扫码

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
u16 USART_STA=0;       //接收状态标记
static u8 MISSION_POINT;     //任务起始区域标志
static u8 PART_AREA;      //目标停车区域标志
static u8 PART_POINT;			//目标停车点
u8 Motor;         //抬升标志
u8 situation[5];  //定位信息数组 

/*第0位：作业线;1-3号作业线；0为待机状态停驻
 ？ 第1位：抬升标识;1抬2降0滞
  第2位：搭载状态标识：0用户车辆未搭载；1用户车辆已搭载
	第3位：目标车位：a-h

	
*/



int main(void)
{

  // 导航信号
	//uint8_t trace_signal_neg=0;// 导航信号判断量
  SystemInit();
	RCC_Configuration();
	GPIO_Configuration();
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);

	//NVIC_Configuration();
	uart_init(115200);
	 
	delay_init();
	
	while(1){


	if(!start_signal)
		//待机状态	
		{
	 	L298N_Stop();
	
		}
			else{	
 // 更新导航信号
  trace_signal = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
	
 	
		
		if(trace_signal==1){
				//检测到高电平，偏向于浅色区域 
			L298N_Stop();
			}
			else if(trace_signal==0){
				//检测到低电平，循迹深色区域
				L298N_Straight();
				
			}
			else{
				//异常，向后台反馈状态
				delay_ms(1000);
				L298N_Stop();
			}
			
			delay_ms(100);
	}
}
	
	
}


void  GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;//声明GPIO初始化结构变量。
	
		//USART2_RX:PA10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//USART2_TX:PA9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //工作频率50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化
	

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;   //配置管脚	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//推挽输出口
	GPIO_Init(GPIOC,&GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   //配置管脚	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//推挽输出口
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6; //配置管脚PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //IO口配置为推挽输出口
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //配置管脚8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //IO口配置为推挽输出口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //工作频率50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   //配置管脚	PA10/USART1_RX 
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP;//推挽输出口
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化


  // RX2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3; //配置管脚PA9/USART1_TX
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //IO口配置为推挽输出口
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //初始化
	
	//TX2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //配置管脚8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //IO口配置为推挽输出口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //工作频率50MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化
	
	//导航传感器管脚PB12、PB13、PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //配置管脚B1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //IO口配置为浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //工作频率50MHz
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化
	
	
	

}

void uart_init(u32 bound){
  //GPIO端口设置
	USART_InitTypeDef USART_InitStructure;
	 

   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	NVIC_Configuration();
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	USART_ClearITPendingBit(USART1,USART_FLAG_ORE);
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
  USART_ClearFlag(USART1, USART_FLAG_ORE);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
 // USART_ITConfig(USART3, USART_IT_TXE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1 
  //USART_ClearFlag(USART3,USART_FLAG_TC);

}



void RCC_Configuration(void){
	SystemInit();
	
	// PA管脚控制车轮
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIOA的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能GPIOA的时钟

	// PB管脚控制传感器
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能GPIOB的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 ,ENABLE);	//使能USART1

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1的时钟， USART1挂接到APB2上。其他USART2-5挂接到APB1上
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 
}


void  ADC_Configuration(void) {
	ADC_InitTypeDef  ADC_InitStructure;
	
	ADC_TempSensorVrefintCmd(ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode  = ADC_Mode_Independent;//选择ADC工作在独立模式或者双ADC模式
	ADC_InitStructure.ADC_ScanConvMode  = ENABLE;  //权责工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode  = ENABLE;//选择ADC工作在连续还是单次模式。

	ADC_InitStructure.ADC_ExternalTrigConv  = ADC_ExternalTrigConv_None;//定义使用外部触发来启动规则通道的ADC
	ADC_InitStructure.ADC_DataAlign  = ADC_DataAlign_Right;//设置数据左对齐还是右对齐
	ADC_InitStructure.ADC_NbrOfChannel  = 1;//设置进行规则转换的ADC通道数目
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);//使能ADC1
	
	ADC_ResetCalibration(ADC1);  // 复位ADC1校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));  //等待校准复位结束

	ADC_StartCalibration(ADC1);//开始校准
	while(ADC_GetCalibrationStatus(ADC1)); //等待校准结束
	
	//ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


//Usart1 NVIC 配置
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
	  //  u8 area_flag; //在该串口中断中，接收的如果是字母，
	// 则将该数据填入任务目标信息。
	// 大写字母代表区域块，
	// 小写字母代表具体位号。
	
		//USART_SendData(USART1, USART_ReceiveData(USART1));

   //  USART_SendData(USART3,3);
     if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断  
    {  
			  USART_ClearFlag(USART1, USART_FLAG_ORE);
			  USART_ClearITPendingBit(USART1,USART_FLAG_ORE); 
        USART_ReceiveData(USART1);			
       // USART_SendData(USART3, USART_ReceiveData(USART3)); 
			  
    }  
		
		
   //正在处理上一条通讯，接收到数据不处理  
    
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
			 //数字开头，是任务起始点信息
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
        while((USART1->SR&0X40)==0);//等待发送完成  
    }
}
/*
void RTC_Configuration(void)

{
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP,ENABLE);
   //使能电源和后备域
    PWR_BackupAccessCmd(ENABLE);
    // 允许访问BKP区域
    BKP_DeInit();
    //BKP复位
    RCC_LSEConfig(RCC_LSE_ON);
    //启动外部低速晶振  32.768k
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    //等待外部低俗晶振是否已经稳定 
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    //选择RTC外部时钟为低俗晶振
    RCC_RTCCLKCmd(ENABLE);
    //使能RTC时钟
    RTC_WaitForSynchro();
    //等待RTC寄存器同步            
    RTC_WaitForLastTask();
    //等待写RTC寄存器完成
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    //使能RTC秒中断       
    RTC_WaitForLastTask();
		RTC_SetPrescaler(32767);
		//设置预分频
		RTC_WaitForLastTask();
}

void RTCClock_Init(void){
	if(BKP_ReadBackupRegister(BKP_DR1)!=0xA5A5){
		//第一次运行  初始化设置
		//RTC初始化
		RTC_Configuration();
		//等待写RTC寄存器完成
		RTC_WaitForLastTask();
    //设置时间初值
		RTC_SetCounter(0x001B82BC);
		RTC_WaitForLastTask();
		//写入配置标志
		BKP_WriteBackupRegister(BKP_DR1,0xA5A5);
	}
	else{
		RTC_WaitForSynchro();
    //等待RTC寄存器同步            
    RTC_WaitForLastTask();
    //等待写RTC寄存器完成
		 RTC_ITConfig(RTC_IT_SEC, ENABLE);
    //使能RTC秒中断   
		RTC_WaitForLastTask();
		
	}
	
	RCC_ClearFlag();    //清除复位标志
}
*/
/*void RTC_IRQHandler(void) {
	if(RTC_GetITStatus(RTC_IT_SEC)!= RESET)//读取中断标志
	{
		RTC_ClearITPendingBit(RTC_IT_SEC);//清除中断标志
	//	tim_bz=1;//秒中断标志
	}
}*/
/*
u16 Get_val(u8 ch)
{
    u16 DataValue; //又是英文注释、、啊哈    
		
			ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
    
   
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
   

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

		
		DataValue = ADC_GetConversionValue(ADC1); 
		return DataValue; 
} 

*/



