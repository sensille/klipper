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
#define F_WRAP(p) ((p) & (F_BUFSZ - 1))
#define F_NEXT(p) F_WRAP((p) + 1)

static struct task_wake fpga_config_wake;
static struct task_wake fpga_serial_wake;
typedef struct fpga_s {
    uint8_t         oid;
    uint8_t         last_oid;   /* cache for responses */
    struct gpio_out clk;
    struct gpio_in miso;
    struct gpio_out mosi;
    struct gpio_out cs;
    struct gpio_out program;
    struct gpio_in init;
    struct gpio_in done;
    struct gpio_out di;
    struct gpio_out serial_reset;
    struct gpio_in serial_error;
    uint8_t         state;
    uint32_t        cnt;       // number of pages sent or end-timer
    uint32_t        uart;
    uint32_t        version;   // FPGA version identifier
    uint8_t         rx_head;
    uint8_t         rx_tail;
    uint8_t         rx_buf[F_BUFSZ];
    uint8_t         rx_seq;
    uint8_t         tx_head;
    uint8_t         tx_tail;
    uint8_t         tx_buf[F_BUFSZ];
    uint8_t         tx_seq;
} fpga_t;

typedef struct {
    uint8_t     msg_id;
    uint8_t     nparams;
} fpga_cmd_t;

static void fpga_rx_byte(void *, uint_fast8_t);
static int fpga_get_tx_byte(void *, uint8_t *);
static void fpga_send(fpga_t *f, fpga_cmd_t *fc, ...);

#define FS_INIT_FLASH       0
#define FS_INIT_FPGA        1
#define FS_INIT_WAIT        2
#define FS_TRANSFER         3
#define FS_SEND_RESPONSE    4
#define FS_INITIALIZED      5

#define CMD_GET_VERSION         0
#define CMD_SYNC_TIME           1
#define CMD_GET_TIME            2
#define CMD_CONFIG_PWM          3
#define CMD_SCHEDULE_PWM        4
#define CMD_CONFIG_STEPPER      5
#define CMD_QUEUE_STEP          6
#define CMD_SET_NEXT_STEP_DIR   7
#define CMD_RESET_STEP_CLOCK    8
#define CMD_STEPPER_GET_POS     9
#define CMD_ENDSTOP_SET_STEPPER 10
#define CMD_ENDSTOP_QUERY       11
#define CMD_ENDSTOP_HOME        12
#define CMD_TMCUART_WRITE       13
#define CMD_TMCUART_READ        14
#define CMD_SET_DIGITAL_OUT     15
#define CMD_CONFIG_DIGITAL_OUT  16
#define CMD_SCHEDULE_DIGITAL_OUT 17
#define CMD_UPDATE_DIGITAL_OUT  18

#define RSP_GET_VERSION         0
#define RSP_GET_TIME            1
#define RSP_STEPPER_GET_POS     2
#define RSP_ENDSTOP_STATE       3
#define RSP_TMCUART_READ        4

fpga_cmd_t cmd_get_version = { CMD_GET_VERSION, 0 };
fpga_cmd_t cmd_sync_time = { CMD_SYNC_TIME, 2 };
fpga_cmd_t cmd_get_uptime = { CMD_GET_TIME, 0 };
fpga_cmd_t cmd_config_pwm = { CMD_CONFIG_PWM, 5 };
fpga_cmd_t cmd_schedule_pwm = { CMD_SCHEDULE_PWM, 3 };
fpga_cmd_t cmd_config_stepper = { CMD_CONFIG_STEPPER, 2 };
fpga_cmd_t cmd_queue_step = { CMD_QUEUE_STEP, 4 };
fpga_cmd_t cmd_set_next_step_dir = { CMD_SET_NEXT_STEP_DIR, 2 };
fpga_cmd_t cmd_reset_step_clock = { CMD_RESET_STEP_CLOCK, 2 };
fpga_cmd_t cmd_stepper_get_pos = { CMD_STEPPER_GET_POS, 1 };
fpga_cmd_t cmd_endstop_set_stepper = { CMD_ENDSTOP_SET_STEPPER, 2 };
fpga_cmd_t cmd_endstop_query = { CMD_ENDSTOP_QUERY, 1 };
fpga_cmd_t cmd_endstop_home = { CMD_ENDSTOP_HOME, 4 };
fpga_cmd_t cmd_tmcuart_write = { CMD_TMCUART_WRITE, 4 };
fpga_cmd_t cmd_tmcuart_read = { CMD_TMCUART_READ, 3 };
fpga_cmd_t cmd_set_digital_out = { CMD_SET_DIGITAL_OUT, 2 };
fpga_cmd_t cmd_config_digital_out = { CMD_CONFIG_DIGITAL_OUT, 4 };
fpga_cmd_t cmd_schedule_digital_out = { CMD_SCHEDULE_DIGITAL_OUT, 3 };
fpga_cmd_t cmd_update_digital_out = { CMD_UPDATE_DIGITAL_OUT, 2 };

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
    fpga_t *f = oid_alloc(args[0], command_config_fpga, sizeof(*f));
    f->oid = args[0];
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
    fpga_t *f = oid_alloc(args[0], command_config_fpga, sizeof(*f));
    f->oid = args[0];
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
    fpga_t *f;
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
            if (!gpio_in_read(f->init)) {
                output("init asserted after %u pages", f->cnt);
                shutdown("fpga config error (init asserted)");
            }
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
    fpga_t *f = oid_lookup(args[0], command_config_fpga);

    serial_setup(args[1], args[2], fpga_rx_byte, fpga_get_tx_byte, f);

    f->uart = args[1];

    // will force output to 1
    configure_timesync_out(args[3]);

    f->serial_reset = gpio_out_setup(args[4], 0);
    f->serial_error = gpio_in_setup(args[5], 0);

    // reset possible serial seq and error
    gpio_out_write(f->serial_reset, 1);
    gpio_out_write(f->serial_reset, 0);

    if (f->rx_head != f->rx_tail)
        output("already got one byte!");

    if (gpio_in_read(f->serial_error))
        shutdown("communication to fpga still in error state");

    // read version
    fpga_send(f, &cmd_get_version);

    // processing continues in the callback for get_version
}
DECL_COMMAND(command_fpga_setup,
    "fpga_setup oid=%c usart_bus=%u rate=%u timesync_pin=%u "
    "serr_pin=%u sreset_pin=%u");

static void
rsp_get_version(fpga_t *f, uint32_t *args)
{
    output("FPGA version is %u", args[0]);

    f->version = args[0];

    // wait for the time to be in the lower half of the 16 bit range
    // We can do this safely from mainloop, as one round only
    // takes ~1.4ms at 48MHz
    while ((timer_read_time() & 0xffff) >= 0x8000)
        ;

    // will set the timesync out to 0 once on next timer overflow
    enable_timesync_out();

    // wait to be in the upper half
    while ((timer_read_time() & 0xffff) < 0x8000)
        ;

    // ... and back in the lower half.
    while ((timer_read_time() & 0xffff) >= 0x8000)
        ;

    // the overflow has occurred. Read the current time, calculate when the
    // wrap happened and send it to the fpga
    uint32_t cur, high;
    read_uptime(&cur, &high);
    cur &= ~0xffff;

    fpga_send(f, &cmd_sync_time, cur, high);
}

void
command_fpga_get_uptime(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[0], command_config_fpga);

    fpga_send(f, &cmd_get_uptime);
}
DECL_COMMAND(command_fpga_get_uptime, "fpga_get_uptime oid=%c");

static void
rsp_get_uptime(fpga_t *f, uint32_t *args)
{
    sendf("fpga_uptime oid=%c high=%u clock=%u", f->oid, args[1], args[0]);
}

typedef struct {
    fpga_t  *fpga;
    uint8_t channel;
} fpga_pwm_t;

void
command_fpga_config_pwm(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[1], command_config_fpga);
    fpga_pwm_t *p = oid_alloc(args[0], command_fpga_config_pwm, sizeof(*p));

    p->fpga = f;
    p->channel = args[2];

    fpga_send(f, &cmd_config_pwm, args[2], args[3], args[4], args[5], args[6]);
}
DECL_COMMAND(command_fpga_config_pwm, "fpga_config_pwm oid=%c fpga_oid=%c "
    "channel=%c cycle-ticks=%u on-ticks=%u default=%c max_duration=%u");

void
command_fpga_schedule_pwm(uint32_t *args)
{
    fpga_pwm_t *p = oid_lookup(args[0], command_fpga_config_pwm);

    fpga_send(p->fpga, &cmd_schedule_pwm, p->channel, args[1], args[2]);
}
DECL_COMMAND(command_fpga_schedule_pwm, "fpga_schedule_pwm oid=%c clock=%u "
    "on-ticks=%u");

typedef struct {
    fpga_t  *fpga;
    uint8_t channel;
    uint8_t slave;
} fpga_tmcuart_t;

void
command_fpga_config_tmcuart(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[1], command_config_fpga);
    fpga_tmcuart_t *t = oid_alloc(args[0], command_fpga_config_tmcuart,
        sizeof(*t));

    t->fpga = f;
    t->channel = args[2];
    t->slave = args[3];
}
DECL_COMMAND(command_fpga_config_tmcuart, "fpga_config_tmcuart oid=%c "
    "fpga_oid=%c channel=%c slave=%c");

void
command_fpga_tmcuart_read(uint32_t *args)
{
    fpga_tmcuart_t *t = oid_lookup(args[0], command_fpga_config_tmcuart);

    t->fpga->last_oid = args[0];

    fpga_send(t->fpga, &cmd_tmcuart_read, t->channel, t->slave, args[1]);
}
DECL_COMMAND(command_fpga_tmcuart_read, "fpga_tmcuart_read oid=%c register=%c");

static void
rsp_tmcuart_read(fpga_t *f, uint32_t *args)
{
    sendf("fpga_tmcuart_data oid=%c status=%c data=%u", f->last_oid,
        args[0], args[1]);
}

void
command_fpga_tmcuart_write(uint32_t *args)
{
    fpga_tmcuart_t *t = oid_lookup(args[0], command_fpga_config_tmcuart);

    fpga_send(t->fpga, &cmd_tmcuart_write, t->channel, t->slave, args[1],
        args[2]);
}
DECL_COMMAND(command_fpga_tmcuart_write,
    "fpga_tmcuart_write oid=%c register=%c data=%u");

typedef struct {
    fpga_t  *fpga;
    uint8_t channel;
} fpga_stepper_t;

void
command_fpga_config_stepper(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[1], command_config_fpga);
    fpga_stepper_t *s = oid_alloc(args[0], command_fpga_config_stepper,
        sizeof(*s));

    s->fpga = f;
    s->channel = args[2];

    fpga_send(f, &cmd_config_stepper, args[2], args[3]);
}
DECL_COMMAND(command_fpga_config_stepper,
    "fpga_config_stepper oid=%c fpga_oid=%c channel=%c dedge=%c");

void
command_fpga_queue_step(uint32_t *args)
{
    fpga_stepper_t *s = oid_lookup(args[0], command_fpga_config_stepper);

    fpga_send(s->fpga, &cmd_queue_step, s->channel, args[1], args[2], args[3]);
}
DECL_COMMAND(command_fpga_queue_step,
    "fpga_queue_step oid=%c interval=%u count=%u add=%u");

void
command_fpga_set_next_step_dir(uint32_t *args)
{
    fpga_stepper_t *s = oid_lookup(args[0], command_fpga_config_stepper);

    fpga_send(s->fpga, &cmd_set_next_step_dir, s->channel, args[1]);
}
DECL_COMMAND(command_fpga_set_next_step_dir,
    "fpga_set_next_step_dir oid=%c interval=%u count=%u add=%u");

void
command_fpga_reset_step_clock(uint32_t *args)
{
    fpga_stepper_t *s = oid_lookup(args[0], command_fpga_config_stepper);

    // Actual stepping starts 2 cycles after the given time
    fpga_send(s->fpga, &cmd_reset_step_clock, s->channel, args[1] - 2);
}
DECL_COMMAND(command_fpga_reset_step_clock,
    "fpga_reset_step_clock oid=%c clock=%u");

void
command_fpga_stepper_get_pos(uint32_t *args)
{
    fpga_stepper_t *s = oid_lookup(args[0], command_fpga_config_stepper);

    s->fpga->last_oid = args[0];

    fpga_send(s->fpga, &cmd_stepper_get_pos, s->channel);
}
DECL_COMMAND(command_fpga_stepper_get_pos,
    "fpga_stepper_get_pos oid=%c");

static void
rsp_stepper_get_pos(fpga_t *f, uint32_t *args)
{
    sendf("fpga_stepper_pos oid=%c position=%u", f->last_oid, args[0]);
}

typedef struct {
    fpga_t          *fpga;
    fpga_stepper_t  *stepper;
    uint8_t         channel;
} fpga_endstop_t;

void
command_fpga_config_endstop(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[1], command_config_fpga);
    fpga_endstop_t *e = oid_alloc(args[0], command_fpga_config_endstop,
        sizeof(*e));

    e->fpga = f;
    e->channel = args[2];
}
DECL_COMMAND(command_fpga_config_endstop,
    "fpga_config_endstop oid=%c fpga_oid=%c channel=%c");

void
command_fpga_endstop_set_stepper(uint32_t *args)
{
    fpga_endstop_t *e = oid_lookup(args[0], command_fpga_config_endstop);
    fpga_stepper_t *s = oid_lookup(args[1], command_fpga_config_stepper);

    fpga_send(e->fpga, &cmd_endstop_set_stepper, e->channel, s->channel);
}
DECL_COMMAND(command_fpga_endstop_set_stepper,
    "fpga_endstop_set_stepper oid=%c stepper_oid=%c");

void
command_fpga_endstop_query(uint32_t *args)
{
    fpga_endstop_t *e = oid_lookup(args[0], command_fpga_config_endstop);

    e->fpga->last_oid = args[0];

    fpga_send(e->fpga, &cmd_endstop_query, e->channel);
}
DECL_COMMAND(command_fpga_endstop_query, "fpga_endstop_query oid=%c");

static void
rsp_endstop_state(fpga_t *f, uint32_t *args)
{
    sendf("fpga_endstop_state oid=%c homing=%c pin_value=%c", f->last_oid,
        args[0], args[1]);
}

void
command_fpga_endstop_home(uint32_t *args)
{
    fpga_endstop_t *e = oid_lookup(args[0], command_fpga_config_endstop);

    fpga_send(e->fpga, &cmd_endstop_home, e->channel, args[1], args[2],
        args[3]);
}
DECL_COMMAND(command_fpga_endstop_home,
    "fpga_endstop_home oid=%c clock=%u sample_count=%c pin_value=%c");

void
command_fpga_set_digital_out(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[0], command_config_fpga);

    fpga_send(f, &cmd_set_digital_out, args[0], args[1]);
}
DECL_COMMAND(command_fpga_set_digital_out,
    "fpga_set_digital_out oid=%c channel=%c pin_value=%c");

typedef struct {
    fpga_t  *fpga;
    uint8_t channel;
} fpga_digital_out_t;
void
command_fpga_config_digital_out(uint32_t *args)
{
    fpga_t *f = oid_lookup(args[1], command_config_fpga);
    fpga_digital_out_t *d = oid_alloc(args[0], command_fpga_config_digital_out,
        sizeof(*d));

    d->fpga = f;
    d->channel = args[2];

    fpga_send(f, &cmd_config_digital_out, args[2], args[3], args[4], args[5]);
}
DECL_COMMAND(command_fpga_config_digital_out,
    "fpga_config_digital_out oid=%c channel=%c value=%c default_value=%c "
    "max_duration=%u");

void
command_fpga_schedule_digital_out(uint32_t *args)
{
    fpga_digital_out_t *d = oid_lookup(args[0],
                                       command_fpga_config_digital_out);

    fpga_send(d->fpga, &cmd_schedule_digital_out, d->channel, args[1], args[2]);
}
DECL_COMMAND(command_fpga_schedule_digital_out,
    "fpga_schedule_digital_out oid=%c clock=%u value=%c");

void
command_fpga_update_digital_out(uint32_t *args)
{
    fpga_digital_out_t *d = oid_lookup(args[0],
                                       command_fpga_config_digital_out);

    fpga_send(d->fpga, &cmd_update_digital_out, d->channel, args[1]);
}
DECL_COMMAND(command_fpga_update_digital_out,
    "fpga_update_digital_out oid=%c value=%c");

struct response_handler_s {
    uint8_t nargs;
    void (*func)(fpga_t *, uint32_t *);
} response_handlers[] = {
    { 1, rsp_get_version },
    { 2, rsp_get_uptime },
    { 1, rsp_stepper_get_pos },
    { 2, rsp_endstop_state },
    { 2, rsp_tmcuart_read },
};
#define RSP_MAX_ID \
    (sizeof(response_handlers) / sizeof(struct response_handler_s))
#define RSP_MAX_ARGS 2

static void
fpga_rx_byte(void *ctx, uint_fast8_t b)
{
    fpga_t *f = ctx;

    if (F_NEXT(f->rx_head) == f->rx_tail)
        shutdown("fpga rx overflow");

    f->rx_buf[f->rx_head] = b;
    f->rx_head = F_NEXT(f->rx_head);

    sched_wake_task(&fpga_serial_wake);
}

static int
fpga_get_tx_byte(void *ctx, uint8_t *b)
{
    fpga_t *f = ctx;

    if (f->tx_head == f->tx_tail)
        return 1;

    *b = f->tx_buf[f->tx_tail];
    f->tx_tail = F_NEXT(f->tx_tail);

    return 0;
}

static void inline
f_writebuf(fpga_t *f, uint8_t ix, uint8_t val)
{
    f->tx_buf[F_WRAP(f->tx_head + ix)] = val;
}

static void
f_encodeint(fpga_t *f, uint32_t v, uint8_t *p)
{
    int32_t sv = v;
    if (sv < (3L<<5)  && sv >= -(1L<<5))  goto f4;
    if (sv < (3L<<12) && sv >= -(1L<<12)) goto f3;
    if (sv < (3L<<19) && sv >= -(1L<<19)) goto f2;
    if (sv < (3L<<26) && sv >= -(1L<<26)) goto f1;
    f_writebuf(f, (*p)++, (v>>28) | 0x80);
f1: f_writebuf(f, (*p)++, ((v>>21) & 0x7f) | 0x80);
f2: f_writebuf(f, (*p)++, ((v>>14) & 0x7f) | 0x80);
f3: f_writebuf(f, (*p)++, ((v>>7) & 0x7f) | 0x80);
f4: f_writebuf(f, (*p)++, v & 0x7f);
}

// Encode a message and send it to the fpga
// messages are only sent and received from main loop, so no sync needed
static void
fpga_send(fpga_t *f, fpga_cmd_t *fc, ...)
{
    va_list args;
    va_start(args, fc);
    uint8_t left;
    uint8_t ptr;
    uint16_t crc = 0xffff;
    int i;

    if (f->tx_tail > f->tx_head)
        left = f->tx_tail - f->tx_head;
    else
        left = f->tx_tail - f->tx_head + F_BUFSZ;

    if (fc->nparams * 5 + 6 > left)
        shutdown("fpga send buffer overrun");

    ptr = 2;
    f_encodeint(f, fc->msg_id, &ptr);
    for (i = 0; i < fc->nparams; ++i)
        f_encodeint(f, va_arg(args, uint32_t), &ptr);
    va_end(args);

    // add frame
    f_writebuf(f, MESSAGE_POS_LEN, ptr + 3);
    f_writebuf(f, MESSAGE_POS_SEQ, 0x10 | ((f->tx_seq++) & 0x0f));
    for (i = 0; i < ptr; ++i)
        crc = crc16_ccitt_one(f->tx_buf[F_WRAP(f->tx_head + i)], crc);
    f_writebuf(f, ptr++, crc >> 8);
    f_writebuf(f, ptr++, crc);
    f_writebuf(f, ptr++, MESSAGE_SYNC);

    f->tx_head = F_WRAP(f->tx_head + ptr);

    serial_enable_tx(f->uart);
}

static uint8_t inline
f_readbuf(fpga_t *f, uint8_t ix)
{
    return f->rx_buf[F_WRAP(f->rx_tail + ix)];
}

static uint32_t
f_parseint(fpga_t *f, uint8_t *ptr)
{
    uint8_t c = f_readbuf(f, (*ptr)++);
    uint32_t v = c & 0x7f;

    if ((c & 0x60) == 0x60)
        v |= -0x20;

    while (c & 0x80) {
        c = f_readbuf(f, (*ptr)++);
        v = (v<<7) | (c & 0x7f);
    }

    return v;
}

void
fpga_serial_task(void)
{
    uint8_t oid;
    fpga_t *f;
    int wake = 0;

    // check if we have a full packet and process it
    int i;

    foreach_oid(oid, f, command_config_fpga) {
        uint8_t rcvd;
        uint8_t l;
        uint8_t ptr;
        uint8_t nargs;
        uint32_t rspid;
        uint32_t args[RSP_MAX_ARGS];
        uint16_t crc = 0xffff;

        if (f->rx_head >= f->rx_tail)
            rcvd = f->rx_head - f->rx_tail;
        else
            rcvd = f->rx_head - f->rx_tail + F_BUFSZ;

        if (rcvd < 1)
            continue;

        l = f_readbuf(f, 0);
        if (l > 32)
            shutdown("bad frame from fpga received (oversized)");
        if (l < 6)
            shutdown("bad frame from fpga received (undersized)");
        if (rcvd < l)
            continue;

        if (f_readbuf(f, l - 1) != MESSAGE_SYNC)
            shutdown("bad frame from fpga received (no sync)");

        for (i = 0; i < l - MESSAGE_TRAILER_SIZE; ++i)
            crc = crc16_ccitt_one(f_readbuf(f, i), crc);

        if (f_readbuf(f, l - MESSAGE_TRAILER_CRC + 0) != (crc >> 8) ||
            f_readbuf(f, l - MESSAGE_TRAILER_CRC + 1) != (crc & 0xff))
            shutdown("bad frame from fpga received (bad crc)");

        if (f_readbuf(f, 1) != (f->rx_seq | 0x10))
            shutdown("bad frame from fpga received (seq)");

        ptr = 2;
        rspid = f_parseint(f, &ptr);
        if (rspid >= RSP_MAX_ID)
            shutdown("bad frame from fpga received (bad response id)");

        nargs = response_handlers[rspid].nargs;
        for (i = 0;  i < nargs; ++i)
            args[i] = f_parseint(f, &ptr);

        ptr += MESSAGE_TRAILER_CRC;
        if (ptr != l)
            shutdown("bad frame from fpga received (param encoding)");
        f->rx_tail = F_WRAP(f->rx_tail + ptr);

        response_handlers[rspid].func(f, args);

        f->rx_seq = (f->rx_seq + 1) & 0x0f;

        if (ptr != rcvd)
            wake = 1;
    }
    if (wake)
        sched_wake_task(&fpga_serial_wake);
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
