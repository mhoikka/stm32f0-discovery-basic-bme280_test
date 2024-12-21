/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_rtc.h"
#include "Peripheral_Init.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rtc.h"
#include "stm32f0xx_exti.h"


/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

/* Private typedef -----------------------------------------------------------*/
struct ambient_reading{
  uint32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
  uint8_t len;
};
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t __IO BlinkSpeed = 0;
/* Private function prototypes -----------------------------------------------*/
extern struct ambient_reading return_sensor_reading(void);  // Declare the function prototype

RCC_ClocksTypeDef RCC_Clocks;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f030.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */ 

  unsigned char data[33] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  int readings_arr[3];

  I2C_Settings_Init();
  UART_Settings_Init();
  System_Clock_Init();

  BME_setup();
  while(!BME_Init()); // Wait for the BME280 to be ready

  NRF24L01p_Init();
  Delay(1); //not 1 ms anymore, closer to 0.34 sec
  while(!test_nrf24_connection()); // Wait for the NRF24 to be ready









RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
PWR_BackupAccessCmd(ENABLE);

RCC_LSICmd(ENABLE); //Enable LSI 
while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); //Wait for LSI to be ready
RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI); //Select LSI as RTC clock source
RCC_RTCCLKCmd(ENABLE);

RTC_WriteProtectionCmd(DISABLE);

RTC_InitTypeDef RTC_InitStruct;
RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
RTC_InitStruct.RTC_AsynchPrediv = 0x7F;
RTC_InitStruct.RTC_SynchPrediv = 0xFF;
RTC_Init(&RTC_InitStruct);



RTC_TimeTypeDef RTC_TimeStruct;
RTC_TimeStruct.RTC_H12 = RTC_H12_AM;
RTC_TimeStruct.RTC_Hours = 0x00;
RTC_TimeStruct.RTC_Minutes = 0x00;
RTC_TimeStruct.RTC_Seconds = 0x00;
RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

RTC_DateTypeDef RTC_DateStruct;
RTC_DateStruct.RTC_WeekDay = 0x01;
RTC_DateStruct.RTC_Month = 0x01;
RTC_DateStruct.RTC_Date = 0x01;
RTC_DateStruct.RTC_Year = 0x00;
RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);


RTC_WaitForSynchro();

//Give alarm interrupt priority
NVIC_InitTypeDef NVIC_InitStruct;
NVIC_InitStruct.NVIC_IRQChannel = RTC_IRQn;
NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStruct);
//Any peripheral interrupt acknowledged by the nested vectored interrupt 
             //controller (NVIC) can wake up the device from Sleep mode.

RTC_TimeTypeDef RTC_TimeStruct_alarm;
RTC_TimeStruct_alarm.RTC_H12 = RTC_H12_AM;
RTC_TimeStruct_alarm.RTC_Hours = 0x00;
RTC_TimeStruct_alarm.RTC_Minutes = 0x00;
RTC_TimeStruct_alarm.RTC_Seconds = 0x10;

char bufery[10];
RTC_AlarmTypeDef RTC_AlarmStruct;
RTC_AlarmStruct.RTC_AlarmTime = RTC_TimeStruct_alarm;
RTC_AlarmStruct.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes; 
RTC_AlarmStruct.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
RTC_AlarmStruct.RTC_AlarmDateWeekDay = 0x01;
RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStruct); 
RTC_ITConfig(RTC_IT_ALRA, ENABLE);
send_stringln(itoa(RTC_AlarmCmd(RTC_Alarm_A, ENABLE), bufery, 10));

RTC_WriteProtectionCmd(ENABLE);











  
  BlinkSpeed = 0;
  //uint8_t powerinc = 0;
  while (1)
  {
    // Display new sensor readings and LED2 Toggle every ~10 seconds
    STM_EVAL_LEDToggle(LED2);
    
    display_sensor_reading();
    struct ambient_reading curr_read = return_sensor_reading();
    readings_arr[0] = (int)curr_read.temperature;
    readings_arr[1] = (int)curr_read.pressure; // value is unsigned int
    readings_arr[2] = (int)curr_read.humidity; // value is unsigned int
    
    transmit(readings_arr, sizeof(readings_arr)/(sizeof(unsigned char)), 1); 

    set_nrf24_SPI_CE(0); //switch NRF24 to standby-I mode by setting CE low

    //Delay(1); // Delay for 10 seconds - BME wakeup time (113 ms max) + NRF24L01+ standby I mode wakeup (130 us)
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Disable SysTick by clearing the ENABLE bit (bit 0)
    PWR_EnterSleepMode(PWR_SLEEPEntry_WFI); //switch STM32 into sleep power mode 
    //MCU should wake up automatically after some time due to RTC alarm
    /*while(powerinc != 29){ //check if system should be re-enabled every loop
      ++powerinc;
      PWR_EnterSleepMode(PWR_SLEEPEntry_WFI); //switch STM32 into sleep power mode 
    }*/
   //wake up from sleep mode

    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick by setting the ENABLE bit (bit 0)
    //powerinc = 0; //reset powerinc to 0
    set_nrf24_SPI_CE(1); //switch NRF24 to TX mode by setting CE high
  }
}

void RTC_IRQHandler(void)
{
    // Check if the interrupt is due to Alarm A
    if (RTC_GetITStatus(RTC_IT_ALRA) != RESET)
    {
        // Clear the interrupt flag
        RTC_ClearITPendingBit(RTC_IT_ALRA);

        // Optionally, you can add other logic here, such as logging the event or resetting counters
    }
}

/**
 * @brief  Initializes the STM32F030 clock
 * @retval None
 */
void System_Clock_Init(){
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency/1000); //SysTick_Config(16777215); // 2^24 - 1 ticks per systick (~0.34 s) // SysTick 1 msec interrupts RCC_Clocks.HCLK_Frequency/1000
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in 1 ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/**
* @brief  function to reverse array
* @param  buffer: array to be reversed
* @param  i: start index
* @param  j: end index
* @retval pointer to reversed char array
*/
char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]); 
	return buffer;
}

/**
* @brief  Iterative function to implement itoa() function in C
* @param  value: value of integer to convert
* @param  buffer: buffer to store the result
* @param  base: base of the result
* @retval pointer to converted character array
*/
char* itoa(int value, char* buffer, int base)
{
	// invalid input
	if (base < 2 || base > 32)
		return buffer; 

	// Get absolute value of number
	int n = value;
    if (n < 0) n *= -1;

	int i = 0;
	while (n)
	{
		int r = n % base;

		if (r >= 10) 
			buffer[i++] = 65 + (r - 10);
		else
			buffer[i++] = 48 + r;

		n = n / base;
	}

	// if number is 0
	if (i == 0)
		buffer[i++] = '0';

	// If base is 10 and value is negative, the resulting string 
	// is preceded with a minus sign (-)
	// With any other base, value is always considered unsigned
	if (value < 0 && base == 10)
		buffer[i++] = '-';

	buffer[i] = '\0'; // null terminate string

	// reverse the string and return it
	return reverse(buffer, 0, i - 1);
}

// We're just going to block for now.
// This is probably never going to DMA
/**
* @brief  Sends a string to over UART 
* @param  string: String to send
* @retval None
*/
void send_string(char *string)
{
    while (*string != 0)
    {
        while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
        USART_SendData(USART2, (uint16_t) *string++);
    }
    while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
}

/**
* @brief  Sends a string to over UART with a newline character
* @param  string: String to send
* @retval None
*/
void send_stringln(char *string)
{
  send_string(string);
  send_string("\r\n");
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
