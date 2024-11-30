#include "stm32f4xx.h"
#include "rfid.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_i2c.h"
#include "ssd1306.h"
#include "fonts.h"
#include <string.h>

#define MAX_RFID_COUNT 10


uint8_t rfid_id[4];
I2C_HandleTypeDef hi2c1;
void SER_Init (void);
void UART4_IRQHandler(void);
int32_t SER_PutChar (int32_t ch);
int32_t SER_GetChar (void);
void mywait(uint32_t count);
void SystemClockConfig (void);
void SER_PutString(char *data);

typedef struct {
    uint8_t rfid_id[4];
    char phone_number[15]; // Store phone number as a string
} RfidPhoneEntry;

// Array of valid RFID entries and corresponding phone numbers
RfidPhoneEntry rfid_entries[MAX_RFID_COUNT] = {
    {{0x03, 0x11, 0xB7, 0x1A}, "+917070197973"},
    {{0xFA, 0x1E, 0x22, 0xB0}, "+919082974873"},
    // Add more entries as needed
};

const char* getPhoneNumber(int index) {
    if (index >= 0 && index < MAX_RFID_COUNT) {
        return rfid_entries[index].phone_number;
    }
    return NULL; // Return NULL if index is invalid
}

int checkRfidInArray(uint8_t rfid_id[4]) {
    for (int i = 0; i < MAX_RFID_COUNT; i++) {
        if (memcmp(rfid_id, rfid_entries[i].rfid_id, 4) == 0) {
            return i;  // Return the index if found
        }
    }
    return -1;  // Return -1 if not found
}

void mywait(uint32_t count) {
    while (count--) {
        __asm("nop"); // Assembly "no operation" to create a simple delay
    }
}

void SER_Init (void){
	#ifdef __DBG_ITM
	ITM_RxBuffer=ITM_RXBUFFER_EMPTY;
	#else
	RCC->APB1ENR|=(1UL<<19);//Enable USART 4 clock
	RCC->AHB1ENR|=(1UL<<2);//Enable GPIOC clock
	GPIOC->MODER &=0XFF0FFFFF;
	GPIOC->MODER |=0X00A00000;
	GPIOC->AFR[1]|=0X00008800;//PC10 UART4_Tx, PC11 UART4_Rx (AF8)
	UART4->BRR=0x1117;
	
	UART4->CR1=0X200C;
	UART4->CR1 |= 0x0020; // Enable RX interrupt
	NVIC_EnableIRQ(UART4_IRQn); // Enable IRQ for UART4 in NVIC 
	#endif
}
int data1, data2;

void UART4_IRQHandler(void) {   
  // RX IRQ part 
	data1=SER_GetChar();
	data2=data1;
}

int32_t SER_PutChar (int32_t ch){
	while (!(UART4->SR& 0X0080));

	UART4->SR&= 0XFFBF;
	UART4->DR=(ch &0xFF);

	return(ch);
}

void SER_PutString(char *data) {
	for (unsigned int j = 0; j < strlen(data); j++) {
			SER_PutChar(data[j]);
	}
}

int32_t SER_GetChar (void){
#ifdef __DBG_ITM
	if(ITM_CheckChar())
		return ITM_ReceiveChar();
#else
	if(UART4->SR&0X0020)
		return ((int32_t) UART4->DR);
#endif

	return(-1);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  /* USER CODE END Error_Handler_Debug */
}

// I2C Initialization for STM32
void I2C1_Init(void) {
		__HAL_RCC_I2C1_CLK_ENABLE();
	
		hi2c1.Instance = I2C1;
		hi2c1.Init.ClockSpeed = 100000;
		hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c1.Init.OwnAddress1 = 0;
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0;
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		{
			Error_Handler();
		}
}

void delay(uint32_t count) {
    while (count--) {
        __asm("nop"); // Assembly "no operation" to create a simple delay
    }
}

void SystemClock_Config (void)
{
	#define PLL_M 4
	#define PLL_N 168
	#define PLL_P 0 //PLL_P = 2
	//ENABLE HSE and wait for the HSE to become Ready
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));

	//Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	//Configure the FLASH PREFETCH and the LATENCY Related Settings
	FLASH->ACR |= FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN|FLASH_ACR_LATENCY_5WS;

	//Configure the PRESCALARS HCLK,PCLK1,PCLK2
	//AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	//APB2 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	//Configure Main PLL
	RCC->PLLCFGR = (PLL_M<<0)|(PLL_N<<6)|(PLL_P<<16)|(RCC_PLLCFGR_PLLSRC_HSE);

	//Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	//Select the Clock source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void SystemClock_Config2(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void SystemClock_Config3(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Enable HSE and wait for it to be ready
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Configure the PLL source (use HSE) and PLL parameters
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;    // Division factor for the HSE
    RCC_OscInitStruct.PLL.PLLN = 168;  // Multiply factor
    RCC_OscInitStruct.PLL.PLLP = 0;    // Division factor for the PLLP (PLLP = 2)
    RCC_OscInitStruct.PLL.PLLQ = 7;    // PLLQ value for USB OTG FS, SDIO

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // Select PLL as the system clock source and configure the AHB, APB1, APB2 clock dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // No division for AHB clock
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   // APB1 clock divider
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   // APB2 clock divider

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)  // Lower latency
    {
        Error_Handler();
    }
}

void GPIO_Config(void)
{
	//Enable Ports clocks
	//__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//Init tyoeDef
	GPIO_InitTypeDef myPinInit;
	//LED pins config
//	myPinInit.Pin = GPIO_PIN_12;
//	myPinInit.Mode = GPIO_MODE_OUTPUT_PP;
//	myPinInit.Pull = GPIO_NOPULL;
//	myPinInit.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOD, &myPinInit);
	//I2C pins config
	myPinInit.Pin = GPIO_PIN_6 |GPIO_PIN_7;
	myPinInit.Mode = GPIO_MODE_AF_OD;
	myPinInit.Pull = GPIO_PULLUP;
	myPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	myPinInit.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &myPinInit);
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Set PA0 as alternate function (AF2 for TIM5)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;  // AF2 for TIM5 CH1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//Systick interrupt enable for HAL_Delay function
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


int test = 0;

int debugToOled(char * deb) {
	ssd1306_Fill(0x00);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(deb, Font_7x10, White);
	ssd1306_UpdateScreen(&hi2c1);
}

void TIM5_Init(void) {
    // Enable TIM5 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    
    // Set prescaler for 1 MHz timer clock (assuming 16 MHz system clock)
    TIM5->PSC = 16 - 1;  // Prescaler value for 1 MHz timer clock (assuming 16 MHz input clock)
    
    // Set the Auto-reload register for 20 ms period (50 Hz frequency) for PWM signal
    TIM5->ARR = 20000;  // PWM period for 50 Hz (20 ms)
    
    // Clear the counter
    TIM5->CNT = 0;
    
    // Configure PWM mode for TIM5 Channel 1 (PA0)
    TIM5->CCMR1 = 0x0060;  // PWM mode for TIM5 CH1 (CC1S = 00, OC1M = 110)
    
    // Enable TIM5 Channel 1 as output
    TIM5->CCER |= TIM_CCER_CC1E;  // Enable channel 1 output
    
    // Set the PWM pulse width to 1.5 ms (neutral position for the servo)
    TIM5->CCR1 = 1500;  // Default to 1.5 ms pulse width for a centered servo (SG90 servo neutral position)
    
    // Enable TIM5 counter
    TIM5->CR1 |= TIM_CR1_CEN;
}

void TIM4_ms_Delay(uint32_t delay) {
    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    
    // Set prescaler for 1 kHz timer clock (assuming 16 MHz system clock)
    TIM4->PSC = 1600000 - 1;
    
    // Set auto-reload for desired delay
    TIM4->ARR = delay;
    
    // Clear the counter
    TIM4->CNT = 0;
    
    // Start the timer
    TIM4->CR1 |= TIM_CR1_CEN;
    
    // Wait for update interrupt flag (end of delay)
    while (!(TIM4->SR & TIM_SR_UIF)) {}
    
    // Clear the interrupt flag
    TIM4->SR &= ~TIM_SR_UIF;
}

int main(void) {
   // Initialize the RC522 RFID module
	HAL_Init();
	GPIO_Config();
	RCC->CFGR |= 0 << 10;  // Set APB1 = 16 MHz
    //GPIO_Init();            // Initialize GPIO for PA0
    TIM5_Init();            // Initialize TIM5 for PWM

    
	SystemClock_Config2();
	I2C1_Init();
  rc522_init();
	SER_Init();
	ssd1306_Init(&hi2c1);

	debugToOled("Scan Your Tag");

	
	while (1) {
		if (rc522_checkCard(rfid_id)) {
			char data[20] = {'\0'};
			sprintf(data, "%02X %02X %02X %02X", (unsigned char)rfid_id[0], (unsigned char)rfid_id[1], (unsigned char)rfid_id[2], (unsigned char)rfid_id[3]);
			debugToOled(data);
			
			int index = checkRfidInArray(rfid_id);
			
			if (index == -1) {
				debugToOled("Invalid Tag");
				TIM4_ms_Delay(3000);
				debugToOled("Scan Your Tag");
				continue;
			}
			
			const char* phone_number = getPhoneNumber(index);
			
			mywait(1000000);
			SystemClock_Config();
	
			SER_PutString("DATA TO SEND\n");
			SER_PutString("AT\n");
			mywait(1000000);
			SER_PutString("AT+CSQ\n");
			mywait(1000000);
			SER_PutString("AT+CCID\n");
			mywait(1000000);
			SER_PutString("AT+CMGF=1\n");
			mywait(1000000);
			SER_PutString("AT+CMGS=\"");
			SER_PutString(phone_number);
			SER_PutString("\"\n");
			mywait(1000000);
			SER_PutString("Thank you for visiting! ID: ");
			SER_PutString(data);
			SER_PutChar(26);
			mywait(1000000);
			SystemClock_Config2();
			mywait(1000000);
			debugToOled("Opening Gate...");
			mywait(1000000);
			TIM5->CCR1 = 680;   // Pulse width for 90 degrees (1.0 ms pulse width)
      TIM4_ms_Delay(10000000);  // Delay for 1 second (open gate)
			TIM4_ms_Delay(1000);  // Delay for 1 second (open gate)
			
			debugToOled("Closing Gate...");
			TIM5->CCR1 = 2000;   // Pulse width for 90 degrees (1.0 ms pulse width)
      TIM4_ms_Delay(1000);  // Delay for 1 second (open gate)
			debugToOled("Scan Your Tag");
		}
	}
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}