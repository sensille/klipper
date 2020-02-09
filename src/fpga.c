// Commands for communicating with a lattice ECP5 FPGA
//
// Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/gpio.h" // gpio_out_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "spicmds.h" // spidev_transfer
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_from_us

static struct task_wake fpga_wake;

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
    uint32_t        cnt;
};

#define FS_INIT_FLASH       0
#define FS_INIT_FPGA        1
#define FS_TRANSFER         2
#define FS_SEND_RESPONSE    3
#define FS_INITIALIZED      4

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline void
ndelay(uint32_t nsecs)
{
    if (CONFIG_CLOCK_FREQ <= 48000000)
        // Slower MCUs don't require a delay
        return;
    uint32_t end = timer_read_time() + nsecs_to_ticks(nsecs);
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

void
command_config_fpga(uint32_t *args)
{
    struct fpga_s *f = oid_alloc(args[0], command_config_fpga,
                                    sizeof(*f));
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

	output("fpga setup called with oid %u", args[0]);

    // wait a 100us before starting
    ndelay(100000);
    f->state = FS_INIT_FLASH;

    sched_wake_task(&fpga_wake);
}
DECL_COMMAND(command_config_fpga,
             "config_fpga oid=%c clk_pin=%u miso_pin=%u mosi_pin=%u cs_pin=%u "
             "program_pin=%u init_pin=%u done_pin=%u di_pin=%u");

void
fpga_task(void)
{
    if (!sched_check_wake(&fpga_wake))
        return;

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
            ndelay(20);
            // check for initn to go high and done low
            if (!gpio_in_read(f->init))
                shutdown("fpga config error (init not high)");
            if (gpio_in_read(f->done))
                shutdown("fpga config error (done not low)");

            f->cnt = 0;
            f->state = FS_TRANSFER;
        } else if (f->state == FS_TRANSFER) {
            // transfer 1024 bits from flash to fpga
            for (i = 0; i < 1024; ++i) {
                gpio_out_write(f->di, gpio_in_read(f->miso));
                ndelay(200);
                gpio_out_write(f->clk, 1);
                ndelay(200);
                gpio_out_write(f->clk, 0);
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
        sched_wake_task(&fpga_wake);
}
DECL_TASK(fpga_task);

#if 0
DECL_SHUTDOWN(spidev_shutdown);
#endif
