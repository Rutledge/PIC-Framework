#include <xc.h>
#include "operations.h"
#include "defines.h"
#include "my_UART.h"
#include "timer.h"

#define MAX_READINGS 200
#define LASER_READSIZE 6
#define MIN_READINGS 29

int readings_num;
static int readings_count;

void move_motor() {
    int target = 2*MAX_READINGS / readings_num;
    LATAbits.LATA1 = 1; // Set PIN for Stepper motor direction high

    start_timer(target, &laser_read);
}

void return_motor() {
    int target = 2*MAX_READINGS;
    LATAbits.LATA1 = 0; // Set PIN for Stepper motor direction low

    start_timer(target, 0);

}

void laser_init() {
    char init = 'U';
    char resp;
    UART2_Write(&init, 1);
    UART2_Read(2);
}

void laser_read() {
    char read[] = "B";
    UART2_Write(&read, 1);
    UART2_Read(LASER_READSIZE);

}

void wifi_write(unsigned long total) {
    char write[5];
    write[0] = total >> 24;
    write[1] = total >> 16;
    write[2] = total >> 8;
    write[3] = total;
    write[4] = '\n';

    UART1_Write(&write, 5);

}

void ReadWIFLY_Message(char* data, int length) {
    if(data[0] < MIN_READINGS || data[0] > MAX_READINGS)
        return;
    readings_num = data[0];
    readings_count = 0;
    UART1_Read_Disable();
    laser_read();
}
// Should not be within int handler
void ReadLaser_Message(char* data, int length) {
    static long lastdist = -1;
    static unsigned long total = 0;
    unsigned long avgdist = 0;
    unsigned long arcarea = 0;

    if (length == LASER_READSIZE) {
        int reading = 0;

        reading += data[2] << 8;
        reading += data[3];

        if (lastdist != -1) {
            avgdist = (reading + lastdist);
            avgdist = (avgdist / 2);
            arcarea = avgdist*avgdist;
            arcarea = (arcarea / readings_num);
            arcarea = (arcarea * 314);
            arcarea = (arcarea / 100);
            total += arcarea;
            readings_count++;
            wifi_write(total);
        }

        lastdist = reading;

        if (readings_count == readings_num) {
            lastdist = -1;
            total = 0;
            return_motor();
            UART1_Read_Enable(0);
        } else {
            move_motor();
        }
    } else {
        UART1_Read_Enable();
    }
}
