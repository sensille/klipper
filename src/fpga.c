// Commands for communicating with a lattice ECP5 FPGA
//
// Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/gpio.h" // gpio_out_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "generic/serial_irq.h"
#include "generic/timer_irq.h" // enable_timesync_out
#include "board/misc.h" // timer_read_time

#if CONFIG_MACH_STM32F0
#include "stm32/internal.h" // gpio_peripheral
#endif

#define F_BUFSZ 128
#define F_NEXT(p) (((p) + 1) & (F_BUFSZ - 1))

static struct task_wake fpga_config_wake;
static struct task_wake fpga_serial_wake;
struct fpga_s {
    struct gpio_out clk;
    struct gpio_in miso;
    struct gpio_out mosi;
    struct gpio_out cs;
    struct gpio_out program;
    struct gpio_in init;
    struct gpio_in done;
    struct gpio_out di;
    uint8_t         state;
    uint32_t        cnt;    /* number of pages sent or end-timer */
    uint32_t        uart;
    uint8_t         rx_head;
    uint8_t         rx_tail;
    uint8_t         rx_buf[F_BUFSZ];
    uint8_t         tx_head;
    uint8_t         tx_tail;
    uint8_t         tx_buf[F_BUFSZ];
};

static void fpga_rx_byte(void *, uint_fast8_t);
static int fpga_get_tx_byte(void *, uint8_t *);

#define FS_INIT_FLASH       0
#define FS_INIT_FPGA        1
#define FS_INIT_WAIT        2
#define FS_TRANSFER         3
#define FS_SEND_RESPONSE    4
#define FS_INITIALIZED      5

static inline uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return (((uint64_t)ns * CONFIG_CLOCK_FREQ) / 1000000000ull);
}

static inline void
ndelay(uint32_t nsecs)
{
    uint32_t end = timer_read_time() + nsecs_to_ticks(nsecs);
    while (timer_is_before(timer_read_time(), end))
        ;
}

void
command_config_fpga(uint32_t *args)
{
    struct fpga_s *f = oid_alloc(args[0], command_config_fpga, sizeof(*f));
    /* spi to flash */
    f->clk = gpio_out_setup(args[1], 0);
    f->miso = gpio_in_setup(args[2], 0);
    f->mosi = gpio_out_setup(args[3], 0);
    f->cs = gpio_out_setup(args[4], 1);
    /* slave serial to fpga */
    f->program = gpio_out_setup(args[5], 0);
    f->init = gpio_in_setup(args[6], 1);
    f->done = gpio_in_setup(args[7], 1);
    f->di = gpio_out_setup(args[8], 1);

    // wait a 100us before starting
    ndelay(100000);
    f->state = FS_INIT_FLASH;

    sched_wake_task(&fpga_config_wake);
}
DECL_COMMAND(command_config_fpga,
             "config_fpga oid=%c clk_pin=%u miso_pin=%u mosi_pin=%u cs_pin=%u "
             "program_pin=%u init_pin=%u done_pin=%u di_pin=%u");

void
command_config_fpga_noinit(uint32_t *args)
{
    struct fpga_s *f = oid_alloc(args[0], command_config_fpga,
                                    sizeof(*f));
    /* slave serial to fpga */
    f->program = gpio_out_setup(args[1], 1);
    f->done = gpio_in_setup(args[2], 1);

    // just check fpga is really in user mode
    if (!gpio_in_read(f->done))
        shutdown("fpga not ready (done not set)");

    f->state = FS_INITIALIZED;
}
DECL_COMMAND(command_config_fpga_noinit,
             "config_fpga_noinit oid=%c program_pin=%u done_pin=%u");

void
fpga_config_task(void)
{
    uint8_t oid;
    struct fpga_s *f;
    int wake = 0;
    int i;

    foreach_oid(oid, f, command_config_fpga) {
        if (f->state == FS_INIT_FLASH) {
            uint32_t data = 0x03000000;

            // setup flash for reading
            gpio_out_write(f->cs, 0);
            for (i = 0; i < 32; ++i) {
                gpio_out_write(f->mosi, data & 0x80000000);
                data <<= 1;
                ndelay(200);
                gpio_out_write(f->clk, 1);
                ndelay(200);
                gpio_out_write(f->clk, 0);
            }

            f->state = FS_INIT_FPGA;
        } else if (f->state == FS_INIT_FPGA) {
            // initialize fpga
            gpio_out_write(f->program, 0);
            ndelay(70);
            // check for initn to go low
            if (gpio_in_read(f->init))
                shutdown("fpga config error (init not low)");
            ndelay(40);
            gpio_out_write(f->program, 1);

            // check for done low
            if (gpio_in_read(f->done))
                shutdown("fpga config error (done not low)");

            f->cnt = timer_read_time() + nsecs_to_ticks(100000000);
            f->state = FS_INIT_WAIT;
        } else if (f->state == FS_INIT_WAIT) {
            // wait up to 100ms for init to go high
            if (timer_is_before(f->cnt, timer_read_time()))
                shutdown("fpga config error (timeout on init)");
            if (gpio_in_read(f->init)) {
                f->cnt = 0;
                f->state = FS_TRANSFER;
            }
        } else if (f->state == FS_TRANSFER) {
            // transfer 1024 bits from flash to fpga
            // both flash and fpga can go beyond 30MHz, so we don't insert any
            // waits here
            int bit;
            for (i = 0; i < 1024; ++i) {
#if CONFIG_MACH_STM32F0
                // optimized version with inlined gpio functions
                ((GPIO_TypeDef *)f->clk.regs)->BSRR = f->clk.bit;
                bit = ((GPIO_TypeDef *)f->miso.regs)->IDR & f->miso.bit;
                ((GPIO_TypeDef *)f->clk.regs)->BSRR = f->clk.bit << 16;
                ((GPIO_TypeDef *)f->di.regs)->BSRR =
                    f->di.bit << (bit ? 0 : 16);
#else
                _gpio_out_write(f->clk, 1);
                bit = _gpio_in_read(f->miso);
                _gpio_out_write(f->clk, 0);
                _gpio_out_write(f->di, bit);
#endif
            }

            // check for error
            if (!gpio_in_read(f->init))
                shutdown("fpga config error (init asserted)");
            // check for done
            if (gpio_in_read(f->done))
                f->state = FS_SEND_RESPONSE;
            // check if max config length is exceeded
            if (++f->cnt >= 16384)
                shutdown("fpga config error (done not set)");
        } else if (f->state == FS_SEND_RESPONSE) {
            sendf("fpga_init_done oid=%c", oid);
            f->state = FS_INITIALIZED;
        }
        if (f->state != FS_INITIALIZED)
            wake = 1;
    }
    if (wake)
        sched_wake_task(&fpga_config_wake);
}

void
command_fpga_setup(uint32_t *args)
{
    struct fpga_s *f = oid_lookup(args[0], command_config_fpga);

    serial_setup(args[1], args[2], fpga_rx_byte, fpga_get_tx_byte, f);

    f->uart = args[1];

    enable_timesync_out(args[3]);

    memcpy(f->tx_buf, "123456789abcdefghijklmnopqrstuvwxyz", 36);
    f->tx_head = 36;
    serial_enable_tx(f->uart);

//    TODO: initiate communication
}
DECL_COMMAND(command_fpga_setup,
    "fpga_setup oid=%c usart_bus=%u rate=%u timesync_pin=%u");

static void
fpga_rx_byte(void *ctx, uint_fast8_t b)
{
    struct fpga_s *f = ctx;

    if (F_NEXT(f->rx_head) == f->rx_tail)
        shutdown("fpga rx overflow");

    f->rx_buf[f->rx_head] = b;
    f->rx_head = F_NEXT(f->rx_head);

    sched_wake_task(&fpga_serial_wake);
}

static int
fpga_get_tx_byte(void *ctx, uint8_t *b)
{
    struct fpga_s *f = ctx;

    *b = 'x';
    return 0;

    if (f->tx_head == f->tx_tail)
        return 1;

    *b = f->tx_buf[f->tx_tail];
    f->tx_tail = F_NEXT(f->tx_tail);

    return 0;
}

void
fpga_serial_task(void)
{
}

void
fpga_task(void)
{
    if (sched_check_wake(&fpga_config_wake))
        fpga_config_task();

    if (sched_check_wake(&fpga_serial_wake))
        fpga_serial_task();
}

DECL_TASK(fpga_task);

#if 0
DECL_SHUTDOWN(spidev_shutdown);
#endif
