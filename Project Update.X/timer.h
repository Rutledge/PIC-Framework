#ifndef TIMER_H
#define TIMER_H

typedef struct {
    int timer_target;
    int count;

    void (*TIMER_Callback) (void);
} TIMER_DATA;

void timer_init(TIMER_DATA *data);
void start_timer(int timer_target, void (*RX_Callback)(void));
void Timer_message_handle();

#endif
