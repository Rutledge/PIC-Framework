#include "interrupts.h"
#include "my_UART.h"
#include "timer.h"
#include <xc.h>
#include "maindefs.h"
#include "messages.h"

//----------------------------------------------------------------------------
// Note: This code for processing interrupts is configured to allow for high and
//       low priority interrupts.  The high priority interrupt can interrupt the
//       the processing of a low priority interrupt.  However, only one of each type
//       can be processed at the same time.  It is possible to enable nesting of low
//       priority interrupts, but this code is not setup for that and this nesting is not
//       enabled.

void enable_interrupts() {
    // Peripheral interrupts can have their priority set to high or low
    // enable high-priority interrupts and low-priority interrupts
    INTCONbits.GIE= 1;         // Set global interrupt
    INTCONbits.PEIE=1;        // Set peripherhal interrupt
}

#ifdef __XC8
// Nothing is needed for this compiler
#else
// Set up the interrupt vectors
void InterruptHandlerHigh();
void InterruptHandlerLow();

#pragma code InterruptVectorLow = 0x18

void
InterruptVectorLow(void) {
    _asm
    goto InterruptHandlerLow //jump to interrupt routine
            _endasm
}

#pragma code InterruptVectorHigh = 0x08

void
InterruptVectorHigh(void) {
    _asm
    goto InterruptHandlerHigh //jump to interrupt routine
            _endasm
}
#endif
//----------------------------------------------------------------------------
// High priority interrupt routine
// this parcels out interrupts to individual handlers

#ifdef __XC8
interrupt
#else
#pragma code
#pragma interrupt InterruptHandlerHigh
#endif
void InterruptHandlerHigh() {
    char data = 0;
    // We need to check the interrupt flag of each enabled high-priority interrupt to
    // see which device generated this interrupt.  Then we can call the correct handler.
    if(PIE1bits.TX1IE && PIR1bits.TX1IF) {
        LATAbits.LATA2 = 1;	// Flag UART 1 Transmit
        UART1_TX_Int_handle();
        LATAbits.LATA2 = 0;	// Unflag UART 1 Transmit
    }
    else if(PIE1bits.RC1IE && PIR1bits.RC1IF) {
        LATAbits.LATA3 = 1;	// Flag UART 1 Recieve
        UART1_RX_Int_handle();
        LATAbits.LATA3 = 0;	// Unflag UART 1 Recieve
    }
    else if(PIE3bits.TX2IE && PIR3bits.TX2IF) {
        LATAbits.LATA4 = 1;	// Flag UART 2 Transmit
        UART2_TX_Int_handle();
        LATAbits.LATA4 = 0;	// Flag UART 2 Transmit
    }
    else if(PIE3bits.RC2IE && PIR3bits.RC2IF) {
        LATAbits.LATA5 = 1;	// Flag UART 2 Recieve
        UART2_RX_Int_handle();
        LATAbits.LATA5 = 0;	// Unlag UART 2 Recieve
    }
    else if(PIE1bits.TMR1IE=1 && PIR1bits.TMR1IF) {
        LATAbits.LATA6 = 1;	// Flag Timer Int
        ToMainLow_sendmsg(1,MSGT_TIMER0, &data);
        PIR1bits.TMR1IF=0;
        LATAbits.LATA6 = 0;	// Unflag Timer Int
    }
}

//----------------------------------------------------------------------------
// Low priority interrupt routine
// this parcels out interrupts to individual handlers
// This works the same way as the "High" interrupt handler
#ifdef __XC8
interrupt low_priority
#else
#pragma code
#pragma interruptlow InterruptHandlerLow
#endif


void InterruptHandlerLow() {
}


int in_high_int() {
    return (!INTCONbits.GIEH);
}

int low_int_active() {
    return (!INTCONbits.GIEL);
}

int in_low_int() {
    if (INTCONbits.GIEL == 1) {
        return (0);
    } else if (in_high_int()) {
        return (0);
    } else {
        return (1);
    }
}

int in_main() {
    if ((!in_low_int()) && (!in_high_int())) {
        return (1);
    } else {
        return (0);
    }
}
