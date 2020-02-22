// STM32F0 timer support
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/armcm_timer.h" // udelay
#include "board/internal.h" // TIM1/2
#include "board/io.h" // readl
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "sched.h" // DECL_INIT
#include "command.h" // DECL_SHUTDOWN
#include "board/timer_irq.h" // timer_dispatch_many


/****************************************************************
 * Low level timer code
 ****************************************************************/

// Use 32bit TIM2 timer if available (otherwise use 16bit TIM1 timer)
#ifdef TIM2
#define TIMx TIM2
#define TIMx_IRQn TIM2_IRQn
#define HAVE_TIMER_32BIT 1
#else
#define TIMx TIM1
#define TIMx_IRQn TIM1_CC_IRQn
#define HAVE_TIMER_32BIT 0
#endif

static inline uint32_t
timer_get(void)
{
    return TIMx->CNT;
}

static inline void
timer_set(uint32_t next)
{
    TIMx->CCR1 = next;
    TIMx->SR = 0;
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    timer_set(timer_get() + 50);
}


/****************************************************************
 * 16bit hardware timer to 32bit conversion
 ****************************************************************/

// High bits of timer (top 17 bits)
static uint32_t timer_high;

// Return the current time (in absolute clock ticks).
uint32_t __always_inline
timer_read_time(void)
{
    if (HAVE_TIMER_32BIT)
        return timer_get();
    uint32_t th = readl(&timer_high);
    uint32_t cur = timer_get();
    // Combine timer_high (high 17 bits) and current time (low 16
    // bits) using method that handles rollovers correctly.
    return (th ^ cur) + (th & 0xffff);
}

// Update timer_high every 0x8000 clock ticks
static uint_fast8_t
timer_event(struct timer *t)
{
    timer_high += 0x8000;
    t->waketime = timer_high + 0x8000;
    return SF_RESCHEDULE;
}
static struct timer wrap_timer = {
    .func = timer_event,
    .waketime = 0x8000,
};

void
timer_reset(void)
{
    if (!HAVE_TIMER_32BIT)
        sched_add_timer(&wrap_timer);
}
DECL_SHUTDOWN(timer_reset);


/****************************************************************
 * Setup and irqs
 ****************************************************************/

// Hardware timer IRQ handler - dispatch software timers
void __aligned(16)
TIMx_IRQHandler(void)
{
    irq_disable();
    uint32_t next = timer_dispatch_many();
    timer_set(next);
    irq_enable();
}

void
timer_init(void)
{
    irqstatus_t flag = irq_save();
    enable_pclock((uint32_t)TIMx);
    TIMx->CNT = 0;
    TIMx->DIER = TIM_DIER_CC1IE;
    TIMx->CCER = TIM_CCER_CC1E;
    armcm_enable_irq(TIMx_IRQHandler, TIMx_IRQn, 2);
    timer_kick();
    timer_reset();
    TIMx->CR1 = TIM_CR1_CEN;
    irq_restore(flag);
}
DECL_INIT(timer_init);

// Route timer overflow to external pin for syncing
void
enable_timesync_out(uint8_t pin)
{
    if (pin != GPIO('B', 14))
        shutdown("invalid timersync pin");

#if TIMx_IRQn != TIM1_CC_IRQn
    shutdown("timersync only possible with timer 1");
#endif

    gpio_peripheral(pin, GPIO_FUNCTION(2) | GPIO_HIGH_SPEED, 0);
#if 0
    gpio_peripheral(GPIO('A', 9), GPIO_FUNCTION(2) | GPIO_HIGH_SPEED, 0);
#endif

    TIMx->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1; // toggle mode
    TIMx->BDTR = TIM_BDTR_MOE;
    TIMx->CCER |= TIM_CCER_CC2NE;
    TIMx->CCER |= TIM_CCER_CC2E;
}
