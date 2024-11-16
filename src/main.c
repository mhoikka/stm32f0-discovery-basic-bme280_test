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
uint8_t NUM_REGISTERS_BME280 = 4;

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
  /*unsigned char data[33] = {0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,      
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                            0xFF, 0xFF, 0x04};  */
unsigned char data[33] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                          0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                          0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                          0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  //unsigned char data[4] = {0x04, 0xFF, 0xFF, 0x04};  

  System_Clock_Init();
  I2C_Settings_Init();
  UART_Settings_Init();

  send_stringln("Start");

  BME_Init();
  bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_initparam);  

  //NRF24L01p_Init();

  Delay(1);
  //test_nrf24_connection();
  //bme280_delay_microseconds(100*1000, NULL);//wait for device to power on

  //transmit(data, sizeof(data)/sizeof(unsigned char));

  STM_EVAL_LEDInit(LED2);
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);  

  BlinkSpeed = 0;

  while (1)
  {
    // Display new sensor readings and LED2 Toggle each 1000ms
    STM_EVAL_LEDToggle(LED2);
    display_sensor_reading();
     
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
 * @brief Display formatted sensor reading from BME280
 * @param pointer to char buffer that stores the sensor readings while they are being written over UART
 * @retval None
 */
struct bme280_data get_sensor_reading(){
  bme280_get_sensor_data(BME280_ALL, &bme280_datastruct, &bme280_initparam);
  return bme280_datastruct;
}

typedef struct my_device_context {
    I2C_InitTypeDef *I2C_InitStruct; // Pointer to the I2C handle
    uint16_t i2c_address;     // I2C address of the BME280
} my_device_context;
struct my_device_context ctx = {};
struct bme280_dev bme280_initparam;
struct bme280_data bme280_datastruct;

/**
 * @brief Initializes the BME 280 sensor
 * @retval None
 */
void BME_Init(){
  bme280_initparam.chip_id = BME280_CHIP_ID; 
  bme280_initparam.intf = BME280_I2C_INTF; 
  bme280_initparam.intf_ptr = (void *)&ctx; 
  bme280_initparam.intf_rslt = BME280_INTF_RET_SUCCESS; 
  bme280_initparam.read = BME280_I2C_bus_read;
  bme280_initparam.write = BME280_I2C_bus_write;
  bme280_initparam.delay_us = bme280_delay_microseconds;
  bme280_init(&bme280_initparam);

  struct bme280_settings bme_settings;
  bme_settings.filter = BME280_FILTER_COEFF_16;             //  Adjust as needed
  bme_settings.osr_h = BME280_OVERSAMPLING_16X ;            // Humidity oversampling
  bme_settings.osr_p = BME280_OVERSAMPLING_16X ;            // Pressure oversampling
  bme_settings.osr_t = BME280_OVERSAMPLING_16X ;            // Temperature oversampling
  bme_settings.standby_time = BME280_STANDBY_TIME_62_5_MS;  // Standby time

  bme280_set_sensor_settings(BME280_SEL_FILTER | BME280_SEL_OSR_HUM | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP, &bme_settings, &bme280_initparam);
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
