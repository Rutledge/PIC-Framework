#include "timer.h"
#include <xc.h>

static TIMER_DATA *timer_data_p;

void timer_init(TIMER_DATA *data) {
    timer_data_p = data;
    PIE1bits.TMR1IE = 1; // Enable time interrupt
    T1CONbits.T1CKPS = 0b11; // Scale timer input by 4
    T1CONbits.TMR1CS = 00; // Scale timer out by 8
    T1GCON = 0;

}

void start_timer(int timer_target, void (*RX_Callback)(void)) {// This function reads in values from the UART recieve bus and returns a reference to the location of the character
    timer_data_p->TIMER_Callback = RX_Callback;
    timer_data_p->timer_target = timer_target;
    timer_data_p->count = 0;

    PIR1bits.TMR1IF = 0;
    TMR1L = 0;
    TMR1H = 0;

    T1CONbits.TMR1ON = 1;

}

void Timer_message_handle() {
    static char wait_status = 0;
    wait_status ^= 1;
    if (wait_status == 0) {
        if (timer_data_p->count == timer_data_p->timer_target) {
            if (*timer_data_p->TIMER_Callback != 0) {
                (*timer_data_p->TIMER_Callback)();
                T1CONbits.TMR1ON = 0;
            }
            LATAbits.LATA0 = 0;
        } else {
            LATAbits.LATA0 ^= 1;
            timer_data_p->count++;
        }
    }

}
