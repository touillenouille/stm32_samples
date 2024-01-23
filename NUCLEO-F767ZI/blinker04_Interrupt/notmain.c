
void PUT32 ( unsigned int, unsigned int );
unsigned int GET32 ( unsigned int );
#define uint32_t	unsigned int
#define RCCBASE		0x40023800
#define RCC_AHB1ENR	(RCCBASE+0x30)
#define RCC_APB1ENR	(RCCBASE+0x40)
#define RCC_APB2ENR	(RCCBASE+0x44)

#define GPIOCBASE	0x40020800
#define GPIOC_MODER     (GPIOCBASE+0x00)
#define GPIOC_PUPDR     (GPIOCBASE+0x0C)
#define GPIOC_IDR       (GPIOCBASE+0x10)

#define GPIOBBASE 	0x40020400
#define GPIOB_MODER     (GPIOBBASE+0x00)
#define GPIOB_BSRR      (GPIOBBASE+0x18)
#define GPIOB_ODR       (GPIOBBASE+0x14)

#define SYSCFGBASE 	0x40013800
#define SYSCFG_EXTICR4	(SYSCFGBASE+0x14)

#define EXTIBASE	0x40013C00
#define EXTI_IMR	(EXTIBASE+0x00)
#define EXTI_RTSR	(EXTIBASE+0x08)
#define EXTI_FTSR	(EXTIBASE+0x0C)
#define EXTI_PR		(EXTIBASE+0x14)

#define NVIC_ISER1      (0xE000E104)
#define NVIC_IPR_BASE	(0xE000E400)

#define WWDG_BASE 0x40002C00
#define WWDG_CR *((volatile unsigned long *)(WWDG_BASE + 0x00))
#define WWDG_CFR *((volatile unsigned long *)(WWDG_BASE + 0x04))
#define WWDG_SR *((volatile unsigned long *)(WWDG_BASE + 0x08))

//PB0
//PB7
//PB14

//PC13

//IRQn_type = 10 -> EXTI4_IRQn

void set_interrupt_priority(uint32_t IRQn, uint32_t priority) {

  uint32_t prio_reg;
  uint32_t prio_pos;
  uint32_t prio_mask;

  if(IRQn >= 0) {
    // Interruption périphérique 
    prio_reg = NVIC_IPR_BASE + (IRQn*4); // Registre de priorité 
    prio_pos = (IRQn % 4) * 8; // Position dans le registre
    prio_mask = 0xFF << prio_pos; // Masque sur 8 bits
  } 
 /* else {
    // Exception système
    prio_reg = SCB_SHPR(IRQn); 
    prio_pos = (IRQn % 4) * 8;
    prio_mask = 0xFF << prio_pos;
  }*/

  // Ecriture de la priorité
  prio_reg &= ~prio_mask; // Clear des bits de priorité
  prio_reg |= (priority << prio_pos) & prio_mask; // Set nouvelle priorité
  PUT32(NVIC_IPR_BASE,prio_reg);
}

static void led_init ( void )
{
    unsigned int ra;

    ra=GET32(RCC_AHB1ENR);
    ra|=1<<1; //enable GPIOB
    PUT32(RCC_AHB1ENR,ra);

    ra=GET32(GPIOB_MODER);
    ra&=~(3<<(0<<1)); //PB0
    ra|= (1<<(0<<1)); //PB0
    ra&=~(3<<(7<<1)); //PB7
    ra|= (1<<(7<<1)); //PB7
    ra&=~(3<<(14<<1)); //PB14
    ra|= (1<<(14<<1)); //PB14
    PUT32(GPIOB_MODER,ra);
}

static void led_on ( void )
{
    PUT32(GPIOB_BSRR,((1<<0)<< 0)|((1<<7)<< 0)|((1<<14)<< 0));
}

static void led_off ( void )
{
    PUT32(GPIOB_BSRR,((1<<0)<<16)|((1<<7)<<16)|((1<<14)<<16));
}

static void toggle_led ( void )
{
    unsigned int ra;
    ra=GET32(GPIOB_ODR);
    if(ra & (1<<0)){
        PUT32(GPIOB_BSRR,((1<<0)<< 16)|((1<<7)<< 16)|((1<<14)<< 16));
    }
    else
    {
	PUT32(GPIOB_BSRR,((1<<0)<<0)|((1<<7)<<0)|((1<<14)<<0));
    }
}

static void button_init ( void )
{
    unsigned int ra;

    ra=GET32(RCC_AHB1ENR);
    ra|=1<<2; //enable GPIOC
    PUT32(RCC_AHB1ENR,ra);

    ra=GET32(GPIOC_MODER);
    ra&=~(3<<26); //PC13
    PUT32(GPIOC_MODER,ra);

    ra=GET32(GPIOC_PUPDR);
    ra&=~(3<<26); //PC13
    PUT32(GPIOC_PUPDR,ra);
	
    ra=GET32(EXTI_IMR);
    ra|=(1<<13);
    PUT32(EXTI_IMR, ra);
//rising trigger
    ra=GET32(EXTI_RTSR);
    ra|=(1<<13);
    PUT32(EXTI_RTSR,ra);

//Falling trigger
/*    ra=GET32(EXTI_FTSR);
    ra|=(1<<13);
    PUT32(EXTI_FTSR,ra);
*/
    ra=GET32(RCC_APB2ENR);
    ra|=(1<<14);
    PUT32(RCC_APB2ENR,ra);

    ra=GET32(SYSCFG_EXTICR4);
    ra|=(0x2<<4);
    PUT32(SYSCFG_EXTICR4,ra);

    ra=GET32(NVIC_ISER1);
    ra=(1 << (40 % 32));  // L'interruption 40 correspond à EXTI15_10
    PUT32(NVIC_ISER1,ra);
    //10 for EXTI15_10 interrupt
    set_interrupt_priority(10,5);
}


void EXTI15_10_IRQHandler(void)
{
    unsigned int ra;
    if(GET32(GPIOC_IDR)&(1<<13)){
        //toggle_led();
	led_on();
    }
    ra=GET32(EXTI_PR);
    ra|=(1<<13);
    PUT32(EXTI_PR,ra);

}

int notmain ( void )
{
    led_init();
    button_init();
    led_off();
    PUT32(WWDG_CFR, (0x7F << 7) | (0x03 << 0));
    while(1){
    PUT32(WWDG_CR, 0x50); 

    for(int i=0; i<100000; i++); 
    }

    return(0);
}
