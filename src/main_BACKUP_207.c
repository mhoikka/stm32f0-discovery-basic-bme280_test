/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"

/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);


/* Private typedef -----------------------------------------------------------*/
typedef struct my_device_context {
    I2C_InitTypeDef *I2C_InitStruct; // Pointer to the I2C handle
    uint16_t i2c_address;     // I2C address of the BME280
} my_device_context;
struct my_device_context ctx = {};
struct bme280_data bme280_datastruct;

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

  char num_buf[65];
  /*
  for(int i = 0; i < sizeof(num_buf)/sizeof(num_buf[0]); i++)
  {
      num_buf[i] = 0;
  }*/

  System_Clock_Init();
  I2C_Settings_Init();
  UART_Settings_Init();

<<<<<<< HEAD
  bme280_init(&bme280);
 
  // Enable peripheral clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx, ENABLE);
  
  // Use pins PA9 and PA10 for I2C (STM32F030R8)
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//code checker is saying this is wrong, not sure why
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1); //GPIO_AF_4
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//do we need this
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // I2C configuration
  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStruct.I2C_DigitalFilter = 0x00;
  I2C_InitStruct.I2C_OwnAddress1 = 0x77;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_Timing = 0x00901D23; //0x10805E89;
  I2C_Init(I2C1, &I2C_InitStruct);
  I2C_Cmd(I2C1, ENABLE);



  while (1) {
         // Read sensor data
         bme280_read_data(&bme280);
         HAL_Delay(1000);
     }
 
=======
  send_stringln("Start");
  struct bme280_dev bme280_initparam;
  bme280_initparam.chip_id = BME280_CHIP_ID; 
  bme280_initparam.intf = BME280_I2C_INTF; 
  bme280_initparam.intf_ptr = (void *)&ctx; 
  bme280_initparam.intf_rslt = BME280_INTF_RET_SUCCESS; 
  bme280_initparam.read = BME280_I2C_bus_read;
  bme280_initparam.write = BME280_I2C_bus_write;
  bme280_initparam.delay_us = bme280_delay_microseconds;
  bme280_init(&bme280_initparam);

  struct bme280_settings bme_settings;
  bme_settings.filter = BME280_FILTER_COEFF_16; // Adjust as needed
  bme_settings.osr_h = BME280_OVERSAMPLING_16X ;       // Humidity oversampling
  bme_settings.osr_p = BME280_OVERSAMPLING_16X ;       // Pressure oversampling
  bme_settings.osr_t = BME280_OVERSAMPLING_16X ;       // Temperature oversampling
  bme_settings.standby_time = BME280_STANDBY_TIME_62_5_MS; // Standby time

  bme280_set_sensor_settings(BME280_SEL_FILTER | BME280_SEL_OSR_HUM | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP, &bme_settings, &bme280_initparam);
  bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280_initparam);
>>>>>>> 4a29ba64667711b675dee03f78e6655a497cf397
  
  Delay (100);
  
  itoa(bme280_get_sensor_data(BME280_ALL, &bme280_datastruct, &bme280_initparam), num_buf, 10);
  send_string(itoa((int)(bme280_datastruct.temperature), num_buf, 10));
  send_stringln(" C");
  send_string(itoa((int)(bme280_datastruct.pressure), num_buf, 10));
  send_stringln(" Pa");
  send_string(itoa((int)(bme280_datastruct.humidity), num_buf, 10));
  send_stringln(" %");

  STM_EVAL_LEDInit(LED2);
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);   
  
  BlinkSpeed = 0;

  while (1)
  {
    STM_EVAL_LEDToggle(LED2);
    // LED2 Toggle each 200ms 
    Delay((1000));
    //send_stringln("Hello, World!");
  }
}


uint8_t NUM_REGISTERS_BME280 = 4;

int8_t BME280_I2C_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr){
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C read
 *  \param *intf_ptr : Pointer to the interface pointer
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : Array for data to be read into from the sensor data register
 *	\param cnt : The # of bytes of data to be read
 **/

	//send the register address to the sensor
	//This is essentially a partial write operation
  uint32_t dev_addr = BME280_I2C_ADDR_SEC;
  //send_stringln("2.1");
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (1 << 16) | (I2C_CR2_AUTOEND);
  //send_stringln("2.3");
	I2C1->TXDR = reg_addr;
  //send_stringln("2.4");
  // Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY));
	while ((I2C1->ISR & I2C_ISR_BUSY));
  //send_stringln("2.5");
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (cnt << 16) | (I2C_CR2_RD_WRN) | (I2C_CR2_AUTOEND);
  //send_stringln("2.6");
	// Transfer all the data
    for (int i = 0; i < cnt; i++) {
        // Wait for RXNE
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        reg_data[i] = I2C1->RXDR;
    }
    //send_stringln("2.7");

	// Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY));
	while ((I2C1->ISR & I2C_ISR_BUSY));
	//return the status of the read operation
  //send_stringln("2.8");
	return 0; //BME OK
}

int8_t BME280_I2C_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr){
/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *  \param *intf_ptr : Pointer to the interface pointer
 *	\param reg_addr : Address of the first register, where data is going to be written
 *	\param reg_data : data to be written into the register
 *	\param cnt : The # of bytes of data to be written
 */
  uint32_t dev_addr = BME280_I2C_ADDR_SEC;
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (cnt*2 << 16) | (I2C_CR2_AUTOEND);
	// transfer cnt bytes of data one register data byte and register address byte pair at a time
	// register address is not auto-incremented
	if (cnt <= NUM_REGISTERS_BME280){
		for(int i = 0; i < cnt; i++){
			// send the register address as a the control byte and the register data as a data byte
			I2C1->TXDR = reg_addr;
			while(!(I2C1->ISR & I2C_ISR_TXE));
			I2C1->TXDR = reg_data[i];
			// increment register address manually
			++reg_addr;
			while(!(I2C1->ISR & I2C_ISR_TXE));
		}
	// Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY));
	while ((I2C1->ISR & I2C_ISR_BUSY));
	return 0;
	}
	else{
		return 1;
		//TODO Create an error function here? Maybe?
	}
}

/**
 * @brief  Delay function for BME280 drivers.
 * @param  usec: specifies the delay time length, in 1 microsecond.
 * @retval None
 */
void __attribute__((optimize("O0"))) bme280_delay_microseconds(uint32_t usec, void *intf_ptr){
  for(volatile uint32_t counter = 0; counter < usec; counter++){
    //do nothing NOP instructions
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();
  }
}

/**
 * @brief  Initializes the I2C3 communication
 * @retval None
 */
void I2C_Settings_Init(){

  // Enable peripheral clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  // Use pins PB6 and PB7 for I2C (STM32F030R8)
  GPIO_InitTypeDef GPIO_InitStruct; 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);//code checker is saying this is wrong, not sure why
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//do we need this
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  // I2C configuration
  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStruct.I2C_DigitalFilter = 0x00;
  I2C_InitStruct.I2C_Timing = 0x00901D23; 
  I2C_Init(I2C1, &I2C_InitStruct);
  I2C_Cmd(I2C1, ENABLE);

  ctx.I2C_InitStruct = &I2C_InitStruct;
  ctx.i2c_address = BME280_I2C_ADDR_SEC; // Set the primary I2C address
}

void  UART_Settings_Init(){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // Is this speed right?
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // No open drain I think
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull up or pull down
  GPIO_Init(GPIOA, &GPIO_InitStruct);


  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &USART_InitStruct);
  //No synchronous mode yet
  USART_Cmd(USART2, ENABLE);
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

// function to reverse buffer[i..j]
char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]); 
	return buffer;
}

// Iterative function to implement itoa() function in C
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
void send_string(char *string)
{
    while (*string != 0)
    {
        while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
        USART_SendData(USART2, (uint16_t) *string++);
    }
    while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
}

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