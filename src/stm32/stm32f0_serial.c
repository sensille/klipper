// STM32F0 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

#define NUSARTS 3

DECL_ENUMERATION("usart_bus", "usart1", 0);
DECL_CONSTANT_STR("BUS_PINS_usart1", "PA10,PA9");
DECL_ENUMERATION("usart_bus", "usart1a", 1);
DECL_CONSTANT_STR("BUS_PINS_usart1a", "PB7,PB6");
DECL_ENUMERATION("usart_bus", "usart2", 2);
DECL_CONSTANT_STR("BUS_PINS_usart2", "PA3,PA2");
DECL_ENUMERATION("usart_bus", "usart2a", 3);
DECL_CONSTANT_STR("BUS_PINS_usart2a", "PA15,PA14");
DECL_ENUMERATION("usart_bus", "usart3", 4);
DECL_CONSTANT_STR("BUS_PINS_usart3", "PB11,PB10");
DECL_ENUMERATION("usart_bus", "usart3a", 5);
DECL_CONSTANT_STR("BUS_PINS_usart3a", "PC5,PC4");

struct usart_bus {
    uint8_t usart;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint8_t af;
};

// mapping for bus-name to usart number and pins
static const struct usart_bus usart_bus[] = {
    {   0, GPIO('A', 10), GPIO('A',  9), 1 },
    {   0, GPIO('B',  7), GPIO('B',  6), 0 },
    {   1, GPIO('A',  3), GPIO('A',  2), 1 },
    {   1, GPIO('A', 15), GPIO('A', 14), 1 },
    {   2, GPIO('B', 11), GPIO('B', 10), 4 },
    {   2, GPIO('C',  5), GPIO('C',  4), 1 },
};
#define NBUS (sizeof(usart_bus) / sizeof(struct usart_bus))

static USART_TypeDef * const usarts[] = { USART1, USART2, USART3 };
static const int irqvecs[] = { USART1_IRQn, USART2_IRQn, USART3_4_IRQn };
static void (*rxfuncs[NUSARTS])(void *, uint_fast8_t);
static int (*txfuncs[NUSARTS])(void *, uint8_t *);
static void *ctxs[NUSARTS];

/* TODO: expose system usart number to host instead of pins */
// Select the configured serial port
#if CONFIG_SERIAL_PORT == 1
  #if CONFIG_STM32_SERIAL_USART1_ALT
    DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB7,PB6");
    #define USART_bus 1
  #else
    DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA10,PA9");
    #define USART_bus 0
  #endif
#elif CONFIG_SERIAL_PORT == 2
  #if CONFIG_STM32_SERIAL_USART2_ALT
    DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA15,PA14");
    #define USART_bus 3
  #else
    DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA3,PA2");
    #define USART_bus 2
  #endif
#endif

#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

static void inline
USARTx_IRQHandler(int n)
{
    uint32_t sr = usarts[n]->ISR;
    if (sr & (USART_ISR_RXNE | USART_ISR_ORE))
        rxfuncs[n](ctxs[n], usarts[n]->RDR);
    if (sr & USART_ISR_TXE && usarts[n]->CR1 & USART_CR1_TXEIE) {
        uint8_t data;
        int ret = txfuncs[n](ctxs[n], &data);
        if (ret)
            usarts[n]->CR1 = CR1_FLAGS;
        else
            usarts[n]->TDR = data;
    }
}

void
USART1_IRQHandler(void)
{
    USARTx_IRQHandler(0);
}

void
USART2_IRQHandler(void)
{
    USARTx_IRQHandler(1);
}

void
USART3_IRQHandler(void)
{
    USARTx_IRQHandler(2);
}

static void
_serial_enable_tx_irq(uint32_t bus)
{
    usarts[usart_bus[bus].usart]->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
}

// specific for command processing
void
serial_enable_tx_irq(void)
{
    _serial_enable_tx_irq(USART_bus);
}

//  generic entry point to enable tx irq
void
serial_enable_tx(uint32_t bus)
{
    _serial_enable_tx_irq(bus);
}

DECL_ARMCM_IRQ(USART1_IRQHandler, USART1_IRQn);
DECL_ARMCM_IRQ(USART2_IRQHandler, USART2_IRQn);
DECL_ARMCM_IRQ(USART3_IRQHandler, USART3_4_IRQn);

void
serial_setup(uint32_t bus, uint32_t rate, void (*rxfunc)(void *, uint_fast8_t),
    int (*txfunc)(void *, uint8_t *), void *ctx)
{
    int n;

    if (bus >= NBUS)
        shutdown("serial bus out of range");

    n = usart_bus[bus].usart;

    enable_pclock((uint32_t)usarts[n]);

    uint32_t pclk = get_pclock_frequency((uint32_t)usarts[n]);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, rate);
    usarts[n]->BRR = (((div / 16) << USART_BRR_DIV_MANTISSA_Pos)
                    | ((div % 16) << USART_BRR_DIV_FRACTION_Pos));
    usarts[n]->CR1 = CR1_FLAGS;
    armcm_enable_declared_irq(irqvecs[n], 0);

    gpio_peripheral(usart_bus[bus].rx_pin, GPIO_FUNCTION(usart_bus[bus].af), 1);
    gpio_peripheral(usart_bus[bus].tx_pin, GPIO_FUNCTION(usart_bus[bus].af), 0);

    rxfuncs[n] = rxfunc;
    txfuncs[n] = txfunc;
    ctxs[n] = ctx;
}

// stubs for command callbacks
static void
_serial_rx_byte(void *ctx, uint_fast8_t b)
{
    serial_rx_byte(b);
}

static int
_serial_get_tx_byte(void *ctx, uint8_t *b)
{
    return serial_get_tx_byte(b);
}

void
serial_init(void)
{
    serial_setup(USART_bus, CONFIG_SERIAL_BAUD, _serial_rx_byte,
        _serial_get_tx_byte, NULL);
}
DECL_INIT(serial_init);
