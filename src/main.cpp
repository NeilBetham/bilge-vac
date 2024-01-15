#include "utils.h"

#include "registers/rcc.h"
#include "registers/gpio.h"
#include "registers/flash.h"
#include "registers/exti.h"
#include "registers/syscfg.h"
#include "registers/core.h"
#include "registers/adc.h"
#include "registers/timer.h"

#include "i2c.h"
#include "stm_pd.h"
#include "usb_pd_controller.h"
#include "status_light.h"
#include "output_en.h"
#include "rtt.h"
#include "time.h"
#include "power_mux.h"
#include "digipot.h"
#include "power_switch.h"
#include "dishy_power.h"



void HardFault_Handler(void) {
  status_light::set_color(1, 1, 1);
  asm("bkpt 1");
  while(1);
}

int main() {
  // Init status light
  // LED Hello World
  status_light::init();
  status_light::set_color(1, 0, 0);

  // Setup core clock to be 64MHz
  // Turn off PLL and wait for it to stop
  RCC_CR &= ~(BIT_24);
  while(RCC_CR & BIT_25);

  // Enable the high speed interal oscillator
  RCC_CR |= BIT_8 | BIT_9;
  while(!(RCC_CR & BIT_10));

  // Set pll source, multiplier and divisior for 64 MHz
  RCC_PLL_CFGR |=   0x00000002;         // Set PLL source to HSI16
  RCC_PLL_CFGR &= ~(0x00000007 << 4);   // Set PLL input scaler M to 1
  RCC_PLL_CFGR &= ~(0x0000007F << 8);   // Set PLL mult factor to 8
  RCC_PLL_CFGR |=   0x00001000 << 8;    // Set PLL mult factor to 8
  RCC_PLL_CFGR |=   BIT_28;             // Enable PLL R output
  RCC_PLL_CFGR &= ~(0x00000007 << 29);  // Set R output scalar to 2
  RCC_PLL_CFGR |=   0x00000001 << 29;   // Set R output scalar to 2

  // Set flash wait states
  FLASH_ACR &= ~(0x7);
  FLASH_ACR |= 0x2;  // 2 wait states for 64 MHz

  // Enable PLL
  RCC_CR |= BIT_24;
  while(RCC_CR & BIT_25);

  // Switch system clock to PLL
  RCC_CFGR &= ~(0x7);
  RCC_CFGR |= 0x2;

  // Init systick
  systick_init();

  // Enable SYSCFG clocks
  RCC_APBENR2 |= BIT_0;

  // Enable A B C D GPIOs
  RCC_IOPENR |= 0xF;

  // Make sure RTT gets inited
  rtt_printf("----------");
  rtt_printf("Booting...");

  status_light::set_color(0, 0, 1);

  // Init PORTB enable lines
  GPIO_B_ODR &= ~(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_13);
  GPIO_B_MODER &= ~(0x0C0000FF);
  GPIO_B_MODER |=  (0x04000055);

  // Enable 24V Supply
  GPIO_D_ODR &= ~(BIT_0);
  GPIO_D_MODER &= ~(0x0000000F);
  GPIO_D_MODER |= 0x000000001;
  GPIO_D_ODR |= (BIT_0);

  // Init the ADC for PA0
  GPIO_A_MODER &= ~(0x00000003);
  GPIO_A_MODER |= 0x00000003;

  // Setup buzzer GPIO
  GPIO_C_MODER &= ~(0x00003000);
  GPIO_C_MODER |= 0x00002000;
  GPIO_C_AFRL &= ~(0x0F000000);
  GPIO_C_AFRL |= 0x01000000;

  // Setup buzzer timer
  RCC_APBENR1 |= BIT_1;
  TIM_3_CCMR1 &= ~(0x00000070);
  TIM_3_CCMR1 |= 0x00000060;
  TIM_3_CCER  |= BIT_0;
  TIM_3_PSC   &= ~(0x0000FFFF);
  TIM_3_PSC   |= 0x00000040;
  TIM_3_ARR   = 250;
  TIM_3_CCR1  = 125;
  TIM_3_CR1   |= BIT_0;



  // Enable ADC
  RCC_AHBENR  |= BIT_0 | BIT_1;
  RCC_APBENR2 |= BIT_20;

  // Enable the ADC vreg and wait for it to stabilize
  ADC_CR |= BIT_28;
  msleep(1);  // Only 20uS are really needed here

  // Run the ADC cal
  ADC_CR |= BIT_31;
  while(ADC_CR & BIT_31);
  ADC_ISR |= BIT_11;

  // Enable the ADC and wait for it to be ready
  ADC_CR |= BIT_0;
  while(!(ADC_ISR & BIT_0));

  // Setup the sampling config
  if(ADC_CFGR1 & BIT_21) {
    ADC_CFGR1 &= ~(BIT_21);  // Single bit channel sampling sequencing
    while(!(ADC_ISR & BIT_13));  // Wait for the CCRDY flag to set
    ADC_ISR |= BIT_13;
  }

  // Channels 1 is selected
  ADC_CHSELR |= BIT_0;
  while(!(ADC_ISR & BIT_13));  // Wait for the CCRDY flag to set
  ADC_ISR |= BIT_13;

  // Set scan direction to low to high
  if(ADC_CFGR1 & BIT_2) {
    ADC_CFGR1 &= ~(BIT_2);
    while(!(ADC_ISR & BIT_13));  // Wait for the CCRDY flag to set
    ADC_ISR |= BIT_13;
  }

  // Setup the sampling time
  ADC_SMPR &= ~(BIT_9 | BIT_10 | BIT_11);
  ADC_SMPR |= (0x7 << BIT0_POS);



  // Enable interrupts
  asm("CPSIE i");

  while(true) {
  	ADC_CR |= BIT_2;
  	while(!(ADC_ISR & BIT_2));
		uint32_t pressure_counts = ADC_DR;

		// Convert counts to a float
		// 12 bits, 3.3v ref, 100 ohm load
		// 3.3 Q16 = 216269
		// 2^16 = 65536
		// 4-20ma => -14.7 -> 30 psig
    // -14.7 psi => 963379 psi q16
		// total psi range = 44.7 psi
		// psi / ma = 2.793 => 183091200 q16
    // 4ma => 262 Q16
		int64_t volts_q16 = (pressure_counts * 216269) >> 12;
		int64_t ma_q16 = (volts_q16 / 100) - 262;
		int64_t psi_q16 = ((ma_q16 * 183091200) >> 16) - 963379;

		rtt_printf("Pressure Counts %d, PSI %d", pressure_counts,  psi_q16 >> 16);

		msleep(100);
  }

  return 0;
}

