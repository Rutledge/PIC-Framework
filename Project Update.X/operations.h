/* 
 * File:   operations.h
 * Author: Rutledge
 *
 * Created on November 30, 2013, 11:53 PM
 */

#ifndef OPERATIONS_H
#define	OPERATIONS_H

void wait_onesec(void);
void move_motor(void);
void laser_init(void);
void laser_read(void);
void wifi_write(unsigned long total);
void ReadWIFLY_Message(char* data, int length);

#endif	/* OPERATIONS_H */

