#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f0xx_spi.h"
#include "Peripheral_Init.h"

typedef struct my_device_context {
    I2C_InitTypeDef *I2C_InitStruct; // Pointer to the I2C handle
    uint16_t i2c_address;     // I2C address of the BME280
} my_device_context;
struct my_device_context ctx = {};
struct bme280_dev bme280_initparam;
struct bme280_data bme280_datastruct;

uint8_t NUM_REGISTERS_BME280 = 4;

uint8_t CONFIG = 0x00;
uint8_t CONFIG_SETTINGS = 0x00;
uint8_t ENAA = 0x01;
uint8_t SETUP_AW = 0x03;
uint8_t RF_SETUP = 0x06;
uint8_t RX_ADDR_P01 = 0x0A;
uint8_t RX_PW_P0 = 0x11;
uint8_t TX_ADDR = 0x10;
uint8_t WRITE_COMMAND = 0x20;
uint8_t WRITE_PAYLOAD_COMMAND = 0xB0; // 0xA0 for ACK
uint8_t READ_PAYLOAD_COMMAND = 0x60;
uint8_t READ_COMMAND = 0x00;
uint8_t STATUS_REG = 0x07;
uint8_t FLUSH_TX = 0xE1;

/**
 * @brief Display formatted sensor reading from BME280
 * @param pointer to char buffer that stores the sensor readings while they are being written over UART
 * @retval None
 */
struct bme280_data get_sensor_reading(){
  bme280_get_sensor_data(BME280_ALL, &bme280_datastruct, &bme280_initparam);
  return bme280_datastruct;
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
    //lol this is nearly perfect timing
  }
}

/**
 * @brief Reads from the I2C bus
 *  @param reg_addr: Address of the first register, where data is going to be read
 * @param reg_data: Pointer to data buffer to store the read data
 * @param cnt: Number of bytes of data to be read
 * @param intf_ptr: Pointer to the interface pointer
 * @retval 0 if successful, 1 if not
 */
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
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (1 << 16) | (I2C_CR2_AUTOEND);
	I2C1->TXDR = reg_addr;

  // Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY));
	while ((I2C1->ISR & I2C_ISR_BUSY));

	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (cnt << 16) | (I2C_CR2_RD_WRN) | (I2C_CR2_AUTOEND);

	// Transfer all the data
    for (int i = 0; i < cnt; i++) {
        // Wait for RXNE
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        reg_data[i] = I2C1->RXDR;
    }


	// Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY));
	while ((I2C1->ISR & I2C_ISR_BUSY));
	//return the status of the read operation

	return 0; //BME OK
}

/**
 * @brief Writes to the I2C bus
 * @param reg_addr: Address of the first register, where data is going to be written
 * @param reg_data: Pointer to data buffer that stores the data to be written
 * @param cnt: Number of bytes of data to be written
 * @param intf_ptr: Pointer to the interface pointer
 * @retval 0 if successful, 1 if not
 */
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

uint8_t* nrf24_read_register(uint8_t reg) {
    uint8_t txData[2]; // Transmit data buffer
    uint8_t rxData[2]; // Receive data buffer

    // Prepare command to read (register address with read command bit)
    txData[0] = reg | READ_COMMAND; // Read command 
    txData[1] = 0x00; // Dummy byte for clocking out data

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission and reception
    for (int i = 0; i < 2; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i];

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte
        rxData[i] = SPI1->DR; // Read received data
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
    return rxData[1]; // Return the value read from the register
}

void nrf24_write_register(uint8_t reg, uint8_t value) {
    uint8_t txData[2]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = reg | WRITE_COMMAND; // Write command
    txData[1] = value;      // Data to write

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    
    // Start SPI transmission
    for (int i = 0; i < 2; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

/** 
* @brief: Writes data to a register
* @param: reg register to write to
* @param: values array of uint8_t data to be transmitted
* @param: num_bytes number of bytes to be transmitted
*/
void nrf24_multiwrite_register(uint8_t reg, uint8_t *values, uint8_t num_bytes) {
    uint8_t txData[num_bytes+1]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = reg | WRITE_COMMAND; // Write command
    memcpy(&txData[1], values, num_bytes); // Copy data to write into buffer

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
    for (int i = 0; i < num_bytes+1; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

/** 
* @brief: Writes a byte of data to the TX FIFO
* @param: value byte of data to be transmitted
*/
//TODO make this capable of transmitting more than one byte
void nrf24_write_TX_payload(uint8_t value) {
    uint8_t txData[2]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = WRITE_PAYLOAD_COMMAND; // Write command
    txData[1] = value;                 // Data to write

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    
    // Start SPI transmission
    for (int i = 0; i < 2; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

/** 
* @brief: Clear TX FIFO
*/
void nrf24_clear_TX(){
    uint8_t txData[1]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = FLUSH_TX; // Write command

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
    for (int i = 0; i < 1; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

void test_nrf24_connection() {
    char num_buf[10];
    char num_buf2[10];
    char num_buf3[10];

    set_nrf24_SPI_CSN(1);
    set_nrf24_SPI_CE(0);
    Delay(100); //Let the chip power up and down

    uint8_t configValue = nrf24_read_register(STATUS_REG); 
    nrf24_write_register(STATUS_REG, CONFIG_SETTINGS); 
    
    uint8_t configValue2 = nrf24_read_register(STATUS_REG); 

    //set_nrf24_SPI_CE(1); //enables chip to receive data
    //Delay(2);

    // Check if expected bits are set
    if ((configValue & CONFIG_SETTINGS) == CONFIG_SETTINGS) {
        send_stringln("Successful: READ bits match WRITE bits");
    } else {
        send_stringln("Failure: READ bits do not match WRITE bits");
    }
}

uint8_t ADDRESS_LEN = 3;
/** 
* @brief: transmits a byte of data for testing purposes
* @param: data byte of data to be transmitted
*/
 //TODO make this much more functional
void transmitByteNRF(uint8_t data){
    uint8_t write_address [3] = {0x93, 0xBD, 0x6B};

    //Clear TX FIFO
    nrf24_clear_TX();
    nrf24_write_register(STATUS_REG, 0x30); //Clear MAX_RT and TX Data Sent bSit from status register
    nrf24_write_register(ENAA, 0x3F); //enable auto ack for all pipes
    
    //set control registers
    nrf24_write_register(SETUP_AW, 0x01); //set to 3 byte address width
    nrf24_multiwrite_register(TX_ADDR, write_address, ADDRESS_LEN); //set write address
    nrf24_multiwrite_register(RX_ADDR_P01, write_address, ADDRESS_LEN); //set read address
    nrf24_write_register(RF_SETUP, 0x00); //set RF Data Rate to 250kbps, RF output power to -18dBm
    nrf24_write_register(RX_PW_P0, 0x01); //set payload size to 1 byte
    //write data to be transmitted into TX FIFO
    nrf24_write_TX_payload(data);
    nrf24_write_register(CONFIG, 0x0A);         //set to PTX mode and turn on power bit
    bme280_delay_microseconds(1.5*1000, NULL);  //wait for chip to go into Standby-I mode

    set_nrf24_SPI_CE(1);                  //enable chip to transmit data
    bme280_delay_microseconds(130, NULL); //wait for chip to go into TX mode
    Delay(1);                             //wait for transmission to complete
    set_nrf24_SPI_CE(0);                  //disable chip after transmission
    nrf24_write_register(CONFIG, 0x08);   //power down by setting PWR_UP bit to 0
}


/**
 * @brief  Enables the CSN pin for the NRF24LO1+ module. Active low
 * @retval None
 */
void set_nrf24_SPI_CSN(uint8_t input){
  if(input == 1){
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
  }
  else{
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
  }
}

/**
 * @brief  Enables the CE pin for the NRF24LO1+ module. Active low
 * @retval None
 */
void set_nrf24_SPI_CE(uint8_t input){
  if(input == 1){
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
  }
  else{
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  }
}

/**
 * @brief Initializes the SPI connection for the STM32F030R8
 * @retval None
 */
void MySPI_Init(){
  //Set up the SPI peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //CHECK IF THIS IS ALREADY ENABLED
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);

  //Set up the SPI pins
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Set up the SPI peripheral
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI1->CR2 |= SPI_CR2_FRXTH; //RXNE event is generated if the FIFO level is greater than or equal to 8

  //Initialize the SPI peripheral
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);
}

/** 
 * @brief Initialize the NRF24L01+ module
 * @retval None
*/  //Not tested
void NRF24L01p_Init(){
  //Set up the SPI peripheral
  MySPI_Init();
  //Set up the GPIO pins
  GPIO_InitTypeDef GPIO_InitStruct_1;
  //Set up the CE and CSN pins
  GPIO_InitStruct_1.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_15;
  GPIO_InitStruct_1.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct_1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_1.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_1.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_Init(GPIOA, &GPIO_InitStruct_1);

  GPIO_InitTypeDef GPIO_InitStruct_2;
  //Set up the IRQ pin
  GPIO_InitStruct_2.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct_2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_2.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct_2.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct_2);
}

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
 * @brief  Initializes the I2C1 communication
 * @retval None
 */
void I2C_Settings_Init(){

  // Enable peripheral clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  // Use pins PB6 and PB7 for I2C (STM32F030R8)
  GPIO_InitTypeDef GPIO_InitStruct; 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
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
  ctx.i2c_address = BME280_I2C_ADDR_SEC; // Set the secondary I2C address
}

/**
 * @brief Initializes the UART connection for the STM32F030R8
 * @retval None
 */
void UART_Settings_Init(){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 
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