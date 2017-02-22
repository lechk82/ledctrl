#include "libmaple/gpio.h"
#include "libmaple/dma.h"
#include "libmaple/timer.h"
#include <SPI.h>

// soft reset stuff
#define SCB_AIRCR ((volatile uint32*) (0xE000ED00 + 0x0C))
#define SCB_AIRCR_SYSRESETREQ (1 << 2)
#define SCB_AIRCR_RESET ((0x05FA0000) | SCB_AIRCR_SYSRESETREQ)

#define pinLED PC13
#define WS2812_DEADPERIOD 19

#define MIN_COLS 1
#define MAX_COLS 255

#define ROWS 16

#define HANDSHAKE 0xaa

typedef union {
  uint16_t val16;
  struct bytes_t {
    unsigned msb : 8;
    unsigned lsb : 8;
  } bytes;
} val16_t;

uint16_t WS2812_IO_H = 0xFFFF;
uint16_t WS2812_IO_L = 0x0000;

volatile uint8_t WS2812_TC = 1;
volatile uint8_t TIM4_overflows = 0;

/* WS2812 framebuffer */
uint16_t *ledbuf;
uint16_t *WS2812_rxbuffer;
uint16_t *WS2812_txbuffer;

volatile uint8_t dma_rx_irq_full_complete = 0;
volatile uint8_t dma_rx_irq_half_complete = 0;

//uint16_t cols;
val16_t cols;
uint32_t led_bufsize = 0;


void DMA1_Channel2_IRQHandler(void){
  uint32_t dma_isr = dma_get_isr_bits(DMA1, DMA_CH2);
  if (dma_isr&DMA_ISR_HTIF1) {
    dma_rx_irq_half_complete = 1;
  }
  if (dma_isr&DMA_ISR_TCIF1) {
    dma_rx_irq_full_complete = 1;
    //WS2812_txbuffer = ledbuf;
  }
  dma_clear_isr_bits(DMA1, DMA_CH2);
  digitalWrite(pinLED, LOW);
}

/* DMA1 Channel7 Interrupt Handler gets executed once the complete framebuffer has been transmitted to the LEDs */
void DMA1_Channel5_IRQHandler(void)
{
  // clear DMA7 transfer complete interrupt flag
  dma_clear_isr_bits(DMA1, DMA_CH5); 
  // enable TIM4 Update interrupt to append 50us dead period
  timer_enable_irq(TIMER4, TIMER_UPDATE_INTERRUPT);
  // disable the DMA channels
  dma_disable(DMA1, DMA_CH7);
  dma_disable(DMA1, DMA_CH4);
  dma_disable(DMA1, DMA_CH5);
  // IMPORTANT: disable the DMA requests, too!
  timer_dma_disable_req(TIMER4, 1);
  timer_dma_disable_req(TIMER4, 2);  
  timer_dma_disable_req(TIMER4, 0); /* TIM_DMA_Update */
}

/* TIM4 Interrupt Handler gets executed on every TIM4 Update if enabled */
void TIM4_IRQHandler(void)
{
  // Clear TIM4 Interrupt Flag
  TIMER4->regs.gen->SR &= ~TIMER_SR_UIF;
  /* check if certain number of overflows has occured yet 
   * this ISR is used to guarantee a 50us dead time on the data lines
   * before another frame is transmitted */
  if (TIM4_overflows < (uint8_t)WS2812_DEADPERIOD)
  {
    // count the number of occured overflows
    TIM4_overflows++;
  }
  else
  {
    // clear the number of overflows
    TIM4_overflows = 0; 
    // stop TIM4 now because dead period has been reached
    timer_pause(TIMER4);
    /* disable the TIM4 Update interrupt again 
     * so it doesn't occur while transmitting data */
    timer_disable_irq(TIMER4, TIMER_UPDATE_INTERRUPT);
    // finally indicate that the data frame has been transmitted
    //WS2812_rxbuffer = ledbuf;
    WS2812_TC = 1;
  }
}

void GPIO_init(void)
{
  // GPIOB Periph clock enable
  rcc_clk_enable(RCC_GPIOB);
  // GPIOB pins WS2812 data outputs
  gpio_set_mode(GPIOB, 0, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 1, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 2, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 3, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 4, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 5, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 6, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 7, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 8, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 9, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 10, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 11, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 12, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 13, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 14, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 15, GPIO_OUTPUT_PP);
}

void TIM4_init(void) {
  uint32_t SystemCoreClock = 72000000;
  uint16_t prescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  rcc_clk_enable(RCC_TIMER4);
  /* Time base configuration */
  timer_pause(TIMER4);
  timer_set_prescaler(TIMER4, prescalerValue);
  timer_set_reload(TIMER4, 29); // 800kHz
  
  /* Timing Mode configuration: Channel 1 */
  timer_set_mode(TIMER4, 2, TIMER_OUTPUT_COMPARE);
  timer_set_compare(TIMER4, 2, 8);
  timer_oc_set_mode(TIMER4, 2, TIMER_OC_MODE_FROZEN, ~TIMER_OC_PE);

  /* Timing Mode configuration: Channel 2 */
  timer_set_mode(TIMER4, 3, TIMER_OUTPUT_COMPARE);
  timer_set_compare(TIMER4, 3, 17);
  timer_oc_set_mode(TIMER4, 3, TIMER_OC_MODE_PWM_1, ~TIMER_OC_PE);
  //timer_resume(TIMER4);


  timer_attach_interrupt(TIMER4, TIMER_UPDATE_INTERRUPT, TIM4_IRQHandler);
  /* configure TIM4 interrupt */
  nvic_irq_set_priority(NVIC_TIMER4, 3);
  nvic_irq_enable(NVIC_TIMER4);
}

void DMA_init(void) {
  dma_init(DMA1);
  dma_setup_transfer( DMA1, 
                      DMA_CH7, 
                      (volatile void*) &(GPIOB->regs->ODR), 
                      DMA_SIZE_16BITS,
                      (volatile void*) &(WS2812_IO_H),
                      DMA_SIZE_16BITS, 
                      DMA_FROM_MEM
                     );
  dma_set_priority(DMA1, DMA_CH7, DMA_PRIORITY_HIGH);

  dma_setup_transfer( DMA1,
                      DMA_CH4,
                      (volatile void*) &(GPIOB->regs->ODR),
                      DMA_SIZE_16BITS,
                      (volatile void*) WS2812_txbuffer,
                      DMA_SIZE_16BITS,
                      DMA_FROM_MEM | DMA_MINC_MODE
                    );
  dma_set_priority(DMA1, DMA_CH4, DMA_PRIORITY_HIGH);

  dma_setup_transfer( DMA1,
                      DMA_CH5,
                      (volatile void*) &(GPIOB->regs->ODR),
                      DMA_SIZE_16BITS,
                      (volatile void*) &(WS2812_IO_L),
                      DMA_SIZE_16BITS,
                      DMA_FROM_MEM | DMA_TRNS_CMPLT
                    );
  dma_set_priority(DMA1, DMA_CH5, DMA_PRIORITY_HIGH);


  /* configure DMA1 Channel7 interrupt */
  nvic_irq_set_priority(NVIC_DMA_CH5, 1);
  nvic_irq_enable(NVIC_DMA_CH5);
  dma_attach_interrupt(DMA1, DMA_CH5, DMA1_Channel5_IRQHandler);
  /* enable DMA1 Channel7 transfer complete interrupt */
}

void ledSend(uint32_t size){   
  // transmission complete flag, indicate that transmission is taking place
  WS2812_TC = 0;
  // clear all relevant DMA flags
  dma_clear_isr_bits(DMA1, DMA_CH7);
  dma_clear_isr_bits(DMA1, DMA_CH4);
  dma_clear_isr_bits(DMA1, DMA_CH5);

  dma_disable(DMA1, DMA_CH7);
  dma_disable(DMA1, DMA_CH4);
  dma_disable(DMA1, DMA_CH5);
  // IMPORTANT: disable the DMA requests, too!
  
  timer_dma_disable_req(TIMER4, 3);
  timer_dma_disable_req(TIMER4, 2);  
  timer_dma_disable_req(TIMER4, 0);
  
  
  // configure the number of bytes to be transferred by the DMA controller
  //dma_set_mem_addr(DMA1, DMA_CH4, WS2812_rxbuffer);
  dma_set_num_transfers(DMA1, DMA_CH7, size);
  dma_set_num_transfers(DMA1, DMA_CH4, size);
  dma_set_num_transfers(DMA1, DMA_CH5, size);

  // clear all TIM4 flags
  TIMER4->regs.gen->SR = 0;
  
  // enable the corresponding DMA channels
  dma_enable(DMA1, DMA_CH7);
  dma_enable(DMA1, DMA_CH4);
  dma_enable(DMA1, DMA_CH5);
  // IMPORTANT: enable the TIM4 DMA requests AFTER enabling the DMA channels!
  timer_dma_enable_req(TIMER4, 3);
  timer_dma_enable_req(TIMER4, 2);
  timer_dma_enable_req(TIMER4, 0); /* TIM_DMA_Update */
  
  // preload counter with 29 so TIM4 generates UEV directly to start DMA transfer
  timer_set_count(TIMER4, 29);
  
  // start TIM4
  timer_resume(TIMER4);
}

void ledSetPixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue, uint16_t *pbuf){
uint8_t i;
uint32_t offset=column*3*8;
uint32_t pos_g;
uint32_t pos_r;
uint32_t pos_b;
uint16_t delmask = ~(0x0001<<row);

  for (i = 0; i < 8; i++){
    pos_g=offset+i;
    pos_r=pos_g+8;
    pos_b=pos_r+8;
    // clear the data for pixel
    pbuf[pos_g] &= delmask;
    pbuf[pos_r] &= delmask;
    pbuf[pos_b] &= delmask;
    // write new data for pixel
    pbuf[pos_g] |= (((((uint16_t)green<<(i+ROWS-8)) & (0x0001<<ROWS-1))>>(ROWS-1))<<row);
    pbuf[pos_r] |= (((((uint16_t)red<<(i+ROWS-8)) & (0x0001<<ROWS-1))>>(ROWS-1))<<row);
    pbuf[pos_b] |= (((((uint16_t)blue<<(i+ROWS-8)) & (0x0001<<ROWS-1))>>(ROWS-1))<<row);
  }
}

void ledWakeUpSequence(void){
uint32_t row;
uint32_t col;
uint8_t val;
  // fade in
  val=0;
    while(val!=255){
    for(row=0;row<ROWS;row++){
      for(col=0;col<cols.val16;col++){
        ledSetPixel(row, col, val, val, val, ledbuf);
      }
    }
    val++;
    ledSend(led_bufsize);
    while(!WS2812_TC);
    
    }
    
    // fade out
  val=255;
    while(val!=0){
      val--;
    for(row=0;row<ROWS;row++){
      for(col=0;col<cols.val16;col++){
        ledSetPixel(row, col, val, val, val, ledbuf);
      }
    }
    ledSend(led_bufsize);
    while(!WS2812_TC);
    }
}

void setup() {
int col,row;
uint8_t initdata;
  pinMode(pinLED, OUTPUT);
  
  Serial.begin(115200);
  delay(100);


  SPI.setModule(1);
  SPI.setDataSize(8);
  SPI.beginSlave();

  Serial.println(F("OK:  Waiting for Handshake..."));
  
  initdata = SPI.read();
  cols.bytes.msb= SPI.read();
  cols.bytes.lsb= SPI.read();
  if(initdata != HANDSHAKE){
    Serial.print(F("ERR: Magic number HANDSHAKE not received (0x"));
    Serial.print(initdata,HEX);
    Serial.print(F(").\n"));
  }
  else{
    Serial.print(F("OK:  Magic number received (0x"));
    Serial.print(initdata,HEX);
    Serial.print(F(").\n"));
  }
   
  if (!((cols.val16 >= MIN_COLS) && (cols.val16 <= MAX_COLS))) {
    Serial.println(F("ERR: #cols not within range. Setting to cols=1 failsafe mode"));
    cols.val16=1;
  }
  else{
    Serial.print(F("OK:  Number of cols received ("));
    Serial.print(cols.val16,DEC);
    Serial.print(F(").\n"));
  }
  

  led_bufsize=cols.val16*8*3;
  ledbuf=(uint16_t *)malloc(led_bufsize);
  if (!ledbuf){
      Serial.println(F("ERR: Allocating memory buffer failed"));
  }
  else{
    Serial.print(F("OK:  "));
    Serial.print(led_bufsize*2,DEC);
    Serial.println(F(" bytes memory buffer allocated."));
  }
  
  SPI.write((unsigned int)HANDSHAKE);
  SPI.read();
  SPI.write((unsigned int)cols.bytes.msb);
  SPI.read(); //dummy read
  SPI.write((unsigned int)cols.bytes.lsb);
  SPI.read(); //dummy read
  
  WS2812_txbuffer = ledbuf;

  GPIO_init();
  DMA_init();
  TIM4_init();
  WS2812_TC=0;
  
  int ret;
  //memset(ledbuf,0,led_bufsize);


  dma_clear_isr_bits(DMA1, DMA_CH2);
  spi_rx_dma_enable(SPI1);
  dma_disable(DMA1, DMA_CH2); // Disable the DMA tube.
  dma_tube_config rx_tube_cfg = {
    &(SPI1->regs->DR),    // data source address
    DMA_SIZE_8BITS,    // source transfer size
    (uint8_t*)ledbuf,               // data destination address 
    DMA_SIZE_8BITS,    // destination transfer size
    led_bufsize*2,      // nr. of data to transfer
    // tube flags: auto increment dest addr, circular rx_buffer, set tube full IRQ, very high prio:
    ( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
    0,  // unused
    DMA_REQ_SRC_SPI1_RX,  // Hardware DMA request source
  };
  ret = dma_tube_cfg(DMA1, DMA_CH2, &rx_tube_cfg);  // SPI1 Rx channel is nr. 2

  dma_attach_interrupt(DMA1, DMA_CH2, DMA1_Channel2_IRQHandler);  // attach an interrupt handler.
  dma_enable(DMA1, DMA_CH2);  // Enable the DMA tube. It will now begin serving requests.
  
  if(ret!=0){
    Serial.println(F("ERR: dma_ch2 spi rx init error"));
  }
  else{
    digitalWrite(pinLED, HIGH);
    Serial.println(F("OK:  Init done."));
  }

}

void loop() {
  while(dma_rx_irq_half_complete==0) digitalWrite(pinLED, HIGH);
  dma_rx_irq_half_complete=0;
  ledSend(led_bufsize);
  while(!WS2812_TC);
}
