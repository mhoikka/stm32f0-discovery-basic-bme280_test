/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f0xx_spi.h"

/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t __IO BlinkSpeed = 0;
/* Private function prototypes -----------------------------------------------*/
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
  /* SysTick end of count event each 1ms */
  /*
  for(int i = 0; i < sizeof(num_buf)/sizeof(num_buf[0]); i++)
  {
      num_buf[i] = 0;
  }*/

  System_Clock_Init();
  I2C_Settings_Init();
  UART_Settings_Init();

  send_stringln("Start");

  //BME_Init();
  //bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_initparam);  

  NRF24L01p_Init();

  Delay(1);
  test_nrf24_connection();

  


  STM_EVAL_LEDInit(LED2);
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);  

  BlinkSpeed = 0;

  while (1)
  {
    // Display new sensor readings and LED2 Toggle each 1000ms
    STM_EVAL_LEDToggle(LED2);
    // display_sensor_reading();
     
    Delay(1000);
  }
}

/**
 * @brief  Initializes the STM32F030R8 clock
 * @retval None
 */
void System_Clock_Init(){
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
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
 * @brief return the sensor reading
 * @retval None
 */
void display_sensor_reading(){
  char num_buf[65];
  struct bme280_data *bme280_datastruct = get_sensor_reading();
  send_string(itoa((int)(bme280_datastruct->temperature), num_buf, 10));
  send_stringln(" C");
  send_string(itoa((int)(bme280_datastruct->pressure), num_buf, 10));
  send_stringln(" Pa");
  send_string(itoa((int)(bme280_datastruct->humidity), num_buf, 10));
  send_stringln(" %");
  send_stringln("");
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
