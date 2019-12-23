/*
 * File:   uart.c
 * Author: michielt
 */
#include <xc.h>
#include "uart.h"

void Uart1SendString(char *buffer ){
    /* Sends string at *buffer until 0x00 char, not adding any other char
     */
    do{
        while( U1STAbits.UTXBF );       // Wait while Transmit buffer is full
        U1TXREG = *buffer;
    }while( *++buffer != 0x00);            // 0x00 is end of string
    while( !U1STAbits.TRMT );           // Transmit Shift Register is not empty and the transmit buffer is not empty
}

void Uart1SendChar(char ch){
    /* Send one char */
             while( U1STAbits.UTXBF );    /* Wait while Transmit buffer is full */
            U1TXREG = ch;
            while( !U1STAbits.TRMT );   /* Transmit Shift Register is not empty and the transmit buffer is not empty */
}