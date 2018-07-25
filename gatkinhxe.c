#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include "pwm.h"

 int16_t tang =0;
 int16_t vitrimoi = 220, loi;
 int16_t vitri2 =0, loi2;
int16_t pwm;
 
int16_t pid(float loi, float kp, float ki, float kd);
void quay(int16_t nangluong);
void debug(char *str, uint16_t dat);

volatile uint32_t system_millis;
 
void sys_tick_handler(void)
{
  system_millis++;
}
 
/* sleep for delay milliseconds */
static void msleep(uint32_t delay)
{
  uint32_t wake = system_millis + delay;
  while (wake > system_millis);
}
 
/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
  /* clock rate / 1000 to get 1mS interrupt rate */
  systick_set_reload(84000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  /* this done last */
  systick_interrupt_enable();
}

 
static void clock_setup(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_USART2);
}
 
static void gpio_setup(void)
{
   gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
   gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
   rcc_periph_clock_enable(RCC_GPIOD);
   gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 |GPIO13| GPIO14);
   rcc_periph_clock_enable(RCC_GPIOA);
   gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,GPIO0 | GPIO3 | GPIO4 | GPIO7);
}
 

 
static void usart_setup(void)
{
 
  usart_set_baudrate(USART2, 9600);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE); 
  usart_enable(USART2);
}
 


static void exti_setup(void)
{
  /* Enable GPIOA clock. */
  rcc_periph_clock_enable(RCC_GPIOA);
 
 
  /* Enable EXTI0 interrupt. */
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);// dung ngat EXTI2
 
  /* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
 
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5 );// CHAN 5 PORTA DOC ENCODER KENH A
 
  /* Configure the EXTI subsystem. */
  exti_select_source(EXTI5, GPIOA);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI5);
}
 
void exti9_5_isr(void)
{
  exti_reset_request(EXTI5); 
  if(gpio_get(GPIOA,GPIO7)==0) // CHAN 7 PORTA DOC TRANG THAI  encoder KENH B
  {
 
    tang--;
  }
 
  else
  {
    tang++;
  }
  //loi = vitrimoi - tang;
  //pwm = pid(loi,5.2,0.12,0.3);
  //quay(pwm);
}

void quay(int16_t nangluong)
{
//  gpio_set(GPIOD,GPIO12);
  if (nangluong >= 50)
  {
    nangluong = 50;
  }
  if (nangluong <= -50)
  {
    nangluong = -50;
  }
  if (nangluong > 0)
  {
    pwm_set_dc(PWM_CH2, 0);// chan 7 PORTB  noi IN2 mach cau H
    pwm_set_dc(PWM_CH3, (uint16_t)nangluong); // chan 8 PORTB  noi IN1 mach cau H
  }
  else if (nangluong < 0)
  {
  //  debug("NL-: ", (uint16_t)nangluong);
    pwm_set_dc(PWM_CH2,(uint16_t)(-nangluong));
    pwm_set_dc(PWM_CH3,0);
  }
  else if (nangluong == 0){
    pwm_set_dc(PWM_CH2,0);
    pwm_set_dc(PWM_CH3,0);
  }
}
 
int16_t pid(float loi, float kp, float ki, float kd)
{
  float dloi;
  static float loitr = 0, iloi =0;
  int16_t temp;
   loitr = loi;
  dloi = loi - loitr;
  iloi += loi;
    if (iloi >= 100)
  {
    iloi = 100;
  }
  if (iloi <= -100)
  {
    iloi = -100;
  }
 
  temp = kp * loi + ki * iloi + kd * dloi;
  if (temp >= 255)
  {
    temp = 255;
  }
  if (temp <= -255)
  {
    temp = -255;
  }
  return temp;
}
uint8_t buff[64];

void debug(char *str, uint16_t dat)
{
  uint8_t ad;
  memset(buff, 0, sizeof(buff));
  sprintf(buff, "%s %d\r\n", str, dat);
  for(ad = 0; ad < strlen(buff); ad++)
    usart_send_blocking(USART2,buff[ad]);
}

void gat_kinh()
{
  loi = vitrimoi - tang;
  quay(pid(loi,2,0.015,0.1));
  if (gpio_get(GPIOA, GPIO4))// CHAN 4 DOC CONG TAC HT  2
          {
            vitrimoi=0;
            loi = vitrimoi - tang;
            quay(pid(loi,2,0.015,0.1));

            while(!gpio_get(GPIOA,GPIO3));// CHAN 3 DOC CONG TAC HT 1
          }
 
   else
   {
      vitrimoi=220;
      quay(pid(loi,2,0.015,0.1));

   }
}

int main(void)
{
  clock_setup();
  gpio_setup();
  systick_setup();
  usart_setup();
  exti_setup();
  pwm_init();
  pwm_start();



  while (1) {
debug("PWM: ", tang);
if(!gpio_get(GPIOA,GPIO0)) // CHAN 0 PORT A DOC CAM BIEN MUA
{
  gpio_set(GPIOD,GPIO12);
  gat_kinh();
}
else
     vitrimoi=-20;
     loi = vitrimoi - tang;
     quay(pid(loi,2,0.015,0.1));
    gpio_clear(GPIOD,GPIO12);
  
   }
 
  return 0;
}