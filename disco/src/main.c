#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>


volatile static uint8_t a[1200];


__attribute__((constructor))
void fun(void)
{
    for (size_t i = 0; i < 1200; i++)
    {
        a[i]= 12;
    }
}

int main(void) {

    for (size_t i = 0; i < 1200; i++)
    {
        printf("%d\n", a[i]);
    }

    return 0;
}