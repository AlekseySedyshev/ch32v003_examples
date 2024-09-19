#include "debug.h"
vu8 val;

#define LED1_ON GPIOC->BCR |= GPIO_Pin_2;
#define LED1_OFF GPIOC->BSHR |= GPIO_Pin_2;

#define LED2_ON GPIOC->BCR |= GPIO_Pin_1;
#define LED2_OFF GPIOC->BSHR |= GPIO_Pin_1;

#define USART_SPEED 115200u

void main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

    // GPIOD->CFGLR |= GPIO_CFGLR_MODE6_0; //GPIO_Mode_IN_FLOATING;

    GPIOD->CFGLR &= (~GPIO_CFGLR_CNF5); // reset pins mode
    GPIOD->CFGLR |= GPIO_CFGLR_CNF5_1;  // Output Alternative PushPull mode
    GPIOD->CFGLR |= GPIO_CFGLR_MODE5;   // Out 50 MHz

    USART1->BRR = (SystemCoreClock / USART_SPEED);
    USART1->CTLR1 |= USART_CTLR1_TE | USART_CTLR1_RE | USART_CTLR1_UE;

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    GPIOC->OUTDR |= GPIO_Pin_1 | GPIO_Pin_2;
    GPIOC->CFGLR &= (~GPIO_CFGLR_CNF1) & (~GPIO_CFGLR_CNF2); // Output mode
    GPIOC->CFGLR |= GPIO_CFGLR_MODE1 | GPIO_CFGLR_MODE2;     // Output mode

    printf("SystemClk:%d\r\n", SystemCoreClock);

    while (1)
    {
        LED1_ON;
        LED2_OFF
        printf("Led1-On, Led2-Off \r\n");
        Delay_Ms(500);
        LED1_OFF;
        LED2_ON;
        printf("Led2-On, Led1-Off \r\n");
        Delay_Ms(500);
    }
}
