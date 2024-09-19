#include <ch32v00x.h>
#include <stdbool.h>


#define BUTTON (GPIOC->INDR & 0x8) // PC3 - Button
#define LED_PIN  0x1  // PC0
#define MOSFET_R 0x4  // PD2
#define MOSFET_L 0x80 // PC7

#define LED_ON GPIOC->OUTDR &= (~LED_PIN)
#define LED_OFF GPIOC->OUTDR |= LED_PIN

#define LED_R_OFF GPIOD->OUTDR &= (~MOSFET_R)
#define LED_R_ON GPIOD->OUTDR |= MOSFET_R

#define LED_L_OFF GPIOC->OUTDR &= (~MOSFET_L)
#define LED_L_ON GPIOC->OUTDR |= MOSFET_L



typedef enum
{
    CH_OFF = 0,
    CH_PRESS = 1,
    CH_RELEASE = 2
} inp_mode;

typedef enum
{
    POW_NOMAL=1,
    POW_SLEEP=2,
    POW_STANDBY=3
} pow_mod;

volatile uint16_t time_press = 0;
volatile uint16_t time_idle = 0;
uint16_t led_tic = 0;
inp_mode but = CH_OFF;

volatile uint16_t TimingDelay; // mSec counter
void DelayMs(uint16_t Delay_time)
{
    TimingDelay = Delay_time;
    while (TimingDelay > 0x00)
    {
    };
}
void deep_sleep(uint8_t sleep_time)
{
    uint8_t ti = 0;
    PWR->CTLR |= PWR_CTLR_PDDS; // StandBy mode
    NVIC_DisableIRQ(SysTicK_IRQn);
    for (ti = 0; ti < sleep_time; ti++)
    {
        PWR->AWUPSC = PWR_AWU_Prescaler_61440;
        PWR->AWUWR = 10;
        PWR->AWUCSR |= 0x2;
        PFIC->SCTLR |= (1 << 2); // Deep Sleep
        __WFE();
        PFIC->SCTLR &= ~(1 << 2);
    }
    NVIC_EnableIRQ(SysTicK_IRQn);
}

void initial_settings(void)
{
    //------------- HSI - 8MHz -------------------
    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = (SystemCoreClock / 1000) - 1; // 1ms
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;

    NVIC_SetPriority(SysTicK_IRQn, 1);
    NVIC_EnableIRQ(SysTicK_IRQn);

    RCC->RSTSCKR |= RCC_LSION;
    while (!(RCC->RSTSCKR & RCC_FLAG_LSIRDY))
        ;

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;
    RCC->APB1PCENR |= RCC_APB1Periph_PWR;

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;
    GPIOA->CFGLR &= (~GPIO_CFGLR_CNF1) & (~GPIO_CFGLR_CNF2); // Set in Analog Input Mode
    GPIOA->CFGLR |= GPIO_CFGLR_CNF1_1 | GPIO_CFGLR_CNF2_1;   // Input Pull Up
    GPIOA->OUTDR = 0x06;                                     // PA1,PA2 Pull Up

    GPIOC->CFGLR &= (~GPIO_CFGLR_CNF1) & (~GPIO_CFGLR_CNF2) & (~GPIO_CFGLR_CNF4) & (~GPIO_CFGLR_CNF5) & (~GPIO_CFGLR_CNF6); // Set all in Analog Input Mode
    GPIOC->CFGLR |= GPIO_CFGLR_CNF1_1 | GPIO_CFGLR_CNF2_1 | GPIO_CFGLR_CNF4_1 | GPIO_CFGLR_CNF5_1 | GPIO_CFGLR_CNF6_1;      // Input Pull Up
    GPIOC->OUTDR |= 0x76;                                                                                                   // PC1,PC2,PC4,PC5,PC6 - Pull Up

    GPIOD->CFGLR &= (~GPIO_CFGLR_CNF1) & (~GPIO_CFGLR_CNF3) & (~GPIO_CFGLR_CNF4) & (~GPIO_CFGLR_CNF5) & (~GPIO_CFGLR_CNF6) & (~GPIO_CFGLR_CNF7); // Set all in Analog Input Mode
    GPIOD->CFGLR |= GPIO_CFGLR_CNF1_1 | GPIO_CFGLR_CNF3_1 | GPIO_CFGLR_CNF4_1 | GPIO_CFGLR_CNF5_1 | GPIO_CFGLR_CNF6_1 | GPIO_CFGLR_CNF7_1;       // Input Pull Up
    GPIOD->OUTDR |= 0xFA;                                                                                                                        // PD4,PD6,PD7 - Pull Up

    //-----------LED--------------

    GPIOC->CFGLR &= (~GPIO_CFGLR_CNF0); // Reset mode
    GPIOC->CFGLR |= GPIO_CFGLR_MODE0;   // Output mode 50 MHZ
    LED_OFF;

    //-------------Output--------------
    LED_L_OFF;
    GPIOC->CFGLR &= (~GPIO_CFGLR_CNF7); // Reset mode
    GPIOC->CFGLR |= GPIO_CFGLR_MODE7;   // PC7 - Output mode 50 MHZ

    LED_R_OFF;
    GPIOD->CFGLR &= (~GPIO_CFGLR_CNF2); // Reset mode
    GPIOD->CFGLR |= GPIO_CFGLR_MODE2;   // PD2 -  Output mode 50 MHZ
                                        //------------input--------------------
    GPIOC->CFGLR &= (~GPIO_CFGLR_CNF3); // Set in Analog Input Mode
    GPIOC->CFGLR |= GPIO_CFGLR_CNF3_1;  // PC3 - Input Pull_Up
    GPIOC->OUTDR |= 0x08;               // PC3 - Pull Up

    AFIO->EXTICR |= 0b10 << 6;

    EXTI->RTENR |= EXTI_RTENR_TR3;
    EXTI->FTENR |= EXTI_FTENR_TR3;
    NVIC_SetPriority(EXTI7_0_IRQn, 4);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
    EXTI->INTENR |= EXTI_INTENR_MR3; // Interrupt Enable

    //---------AWU---------
    EXTI->FTENR |= EXTI_FTENR_TR9;
    NVIC_SetPriority(AWU_IRQn, 5);
    NVIC_EnableIRQ(AWU_IRQn);
    EXTI->INTENR |= EXTI_INTENR_MR9; // Interrupt Enable
}
//---------------------------

void main(void)
{
    initial_settings();
    LED_ON;
    DelayMs(500);
    LED_OFF;
    while (1)
    {
        LED_L_ON;
        DelayMs(500);
        LED_L_OFF;
        DelayMs(500);
        LED_R_ON;
        DelayMs(500);
        LED_R_OFF;
        DelayMs(500);
        deep_sleep(2);
    }
}
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    if (TimingDelay > 0)
    {
        TimingDelay--;
    }

    if (led_tic > 0)
        led_tic--;
    else
    {
        led_tic = 1000;
    }
    if (but == CH_PRESS && time_press < 0xffff)
        time_press++;
    if (but == CH_OFF && time_idle < 0xffff)
        time_idle++;

    SysTick->SR = 0;
}
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI->INTFR & EXTI_Line3) // PC3 - Input
    {
        if ((BUTTON) && (but == CH_OFF || but == CH_RELEASE)) // Activate
        {
            but = CH_PRESS;
        }
        if (!(BUTTON) && but == CH_PRESS) // Diactivate
        {
            but = CH_RELEASE;
        }
        EXTI->INTFR |= EXTI_Line3; /* Clear Flag */
    }
}
void AWU_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void AWU_IRQHandler(void)
{
    EXTI->INTFR |= EXTI_Line9; /* Clear Flag */
    PWR->AWUCSR = 0;
}