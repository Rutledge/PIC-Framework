/* 
 * File:   my_UART.h
 * Author: Rutledge
 *
 * Created on November 24, 2013, 8:03 PM
 */

#ifndef MY_UART_H
#define	MY_UART_H

#define MAXUARTBUF 10

typedef struct {
    char buffer_in[MAXUARTBUF];
    int buffer_in_ind;
    int buffer_in_len;

    char buffer_out[MAXUARTBUF];
    int buffer_out_ind;
    int buffer_out_len;
} UART_DATA;

void UART1_Init(UART_DATA *data);
void UART2_Init(UART_DATA *data);
void UART1_TX_Int_handle();
void UART1_RX_Int_handle();
void UART2_TX_Int_handle();
void UART2_RX_Int_handle();

void UART1_Write(char* c, int length);
void UART2_Write(char* c, int length);

int UART1_Read(char* c);
void UART2_Read(int len);

void UART1_Read_Enable();
void UART1_Read_Disable();

void ReadLaser_Message(char* data, int length);

#endif	/* MY_UART_H */

