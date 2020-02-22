#ifndef __GENERIC_TIMER_IRQ_H
#define __GENERIC_TIMER_IRQ_H

uint32_t timer_dispatch_many(void);

void enable_timesync_out(uint8_t pin);

#endif // timer_irq.h
