#include <xc.h>
#include "my_UART.h"
#include "defines.h"
#include "messages.h"
#include "maindefs.h"



static UART_DATA *uart_1_data_p;

void UART1_Init(UART_DATA *data) {
    uart_1_data_p = data;

    UART1_TX_TRIS = 0; // Tx pin set to output
    UART1_RX_TRIS = 1; // Rx pin set to input

    BAUDCON1bits.BRG16 = 0; // 8-bit baud rate generator
    SPBRG1 = 77; // Set to 25 for UART speed to 115200 baud, 77 for 38400
    TXSTA1bits.BRGH = 1; // High speed mode
    TXSTA1bits.SYNC = 0; // Async mode
    RCSTA1bits.SPEN = 1; // Serial port enable
    TXSTA1bits.TX9 = 0; // 8 bit transmission
    RCSTA1bits.RX9 = 0; // 8 bit reception
    RCSTA1bits.CREN = 1; // Continuous receive mode

    TXSTA1bits.TXEN = 1; // Enable transmission


    // Initialize the buffer that holds UART messages
    uart_1_data_p->buffer_in_ind = 0;
    uart_1_data_p->buffer_out_ind = 0;


}

static UART_DATA *uart_2_data_p;

void UART2_Init(UART_DATA *data) {
    uart_2_data_p = data;

    UART2_RX_TRIS=1;
    UART2_TX_TRIS=0; // Set up UART 2

    BAUDCON2bits.BRG16 = 0; // 8-bit baud rate generator
    SPBRG2 = 77; // Set to 25 for UART speed to 115200 baud, 77 for 38400
    TXSTA2bits.BRGH = 1; // High speed mode
    TXSTA2bits.SYNC = 0; // Async mode
    RCSTA2bits.SPEN = 1; // Serial port enable
    TXSTA2bits.TX9 = 0; // 8 bit transmission
    RCSTA2bits.RX9 = 0; // 8 bit reception
    RCSTA2bits.CREN = 1; // Continuous receive mode

    TXSTA2bits.TXEN = 1; // Enable transmission


    // Initialize the buffer that holds UART messages
    uart_2_data_p->buffer_in_len = -1;
    uart_2_data_p->buffer_in_ind = 0;
    uart_2_data_p->buffer_out_ind = 0;
}

void UART1_Write(char* c, int length){
    if(uart_1_data_p->buffer_out_ind!=0) {
        return;
    }

    uart_1_data_p->buffer_out_len= length;
    for(int i=0; i<length; i++) {
        uart_1_data_p->buffer_out[i] = c[i];
    }
    PIE1bits.TX1IE=1;
   
   
}

void UART2_Write(char* c, int length){
    if(uart_2_data_p->buffer_out_ind!=0){
        return;
    }

    uart_2_data_p->buffer_out_len= length;
     for(int i=0; i<length; i++) {
        uart_2_data_p->buffer_out[i] = c[i];
     }
    PIE3bits.TX2IE=1;
}


void UART1_Read_Enable() {
    PIE1bits.RC1IE = 1;
    char data = RC1REG;
}

void UART1_Read_Disable() {
    PIE1bits.RC1IE = 0;
}

void UART2_Read(int len){// This function reads in values from the UART recieve bus and returns a reference to the location of the character
    if(uart_2_data_p->buffer_in_len != -1){
        return;
    }
    uart_2_data_p->buffer_in_len=len;
    PIE3bits.RC2IE = 1;

}

void UART1_TX_Int_handle() {
    if(uart_1_data_p->buffer_out_len == uart_1_data_p->buffer_out_ind){
        PIE1bits.TX1IE=0;
        uart_1_data_p->buffer_out_ind=0;
//        LATAbits.LATA3 = 0;
    }
    else {
        TXREG1 = uart_1_data_p->buffer_out[uart_1_data_p->buffer_out_ind];
        uart_1_data_p->buffer_out_ind++;
//        LATAbits.LATA3 = 1;
    }

}
void UART1_RX_Int_handle() {
    unsigned char data = RCREG1;
    ToMainHigh_sendmsg(1, MSGT_WIFLY_RECIEVE, &data);

}

void UART2_TX_Int_handle(){
    if(uart_2_data_p->buffer_out_len == uart_2_data_p->buffer_out_ind){
    PIE3bits.TX2IE=0;
    uart_2_data_p->buffer_out_ind=0;
    }

    else {
        TXREG2 = uart_2_data_p->buffer_out[uart_2_data_p->buffer_out_ind];
        uart_2_data_p->buffer_out_ind++;
    }


}

void UART2_RX_Int_handle(){
    uart_2_data_p->buffer_in[uart_2_data_p->buffer_in_ind] = RCREG2;
    uart_2_data_p->buffer_in_ind++;

    if(uart_2_data_p->buffer_in_len == uart_2_data_p->buffer_in_ind){
        PIE3bits.RC2IE=0;
        uart_2_data_p->buffer_in_ind=0;
        ToMainHigh_sendmsg(uart_2_data_p->buffer_in_len, MSGT_LASER_READ, uart_2_data_p->buffer_in);
        uart_2_data_p->buffer_in_len = -1;
    }

}

