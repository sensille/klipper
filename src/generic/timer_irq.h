#ifndef __GENERIC_TIMER_IRQ_H
#define __GENERIC_TIMER_IRQ_H

uint32_t timer_dispatch_many(void);

void configure_timesync_out(uint8_t pin);
void enable_timesync_out(void);

#endif // timer_irq.h
