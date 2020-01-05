// Commands for communicating with a lattice ECP5 FPGA
//
// Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h" // gpio_out_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "spicmds.h" // spidev_transfer

struct fpga_s {
    struct gpio_out program;
    struct gpio_out init;
    struct gpio_out done;
    struct gpio_out hold;
    uint8_t spi_bus;
    uint8_t flash;
};

void
command_config_fpga(uint32_t *args)
{
    struct fpga_s *fpga = oid_alloc(args[0], command_config_fpga,
                                    sizeof(*fpga));
    fpga->flash = args[1];
    fpga->program = gpio_out_setup(args[2], 1);
    fpga->init = gpio_out_setup(args[3], 1);
    fpga->done = gpio_out_setup(args[4], 1);
    fpga->hold = gpio_out_setup(args[5], 1);
    fpga->spi_bus = args[6];

	output("fpga setup called with oid %u", args[0]);
}
DECL_COMMAND(command_config_fpga,
             "config_fpga oid=%c flash_spi=%u program_pin=%u init_pin=%u "
             " done_pin=%u hold_pin=%u spi_bus=%u");

#if 0
struct spidev_s *
spidev_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_spi);
}

void
command_spi_set_bus(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    uint8_t mode = args[2];
    if (mode > 3 || spi->flags & (SF_SOFTWARE|SF_HARDWARE))
        shutdown("Invalid spi config");
    spi->spi_config = spi_setup(args[1], mode, args[3]);
    spi->flags |= SF_HARDWARE;
}
DECL_COMMAND(command_spi_set_bus,
             "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u");

void
spidev_set_software_bus(struct spidev_s *spi, struct spi_software *ss)
{
    if (spi->flags & (SF_SOFTWARE|SF_HARDWARE))
        shutdown("Invalid spi config");
    spi->spi_software = ss;
    spi->flags |= SF_SOFTWARE;
}

void
spidev_transfer(struct spidev_s *spi, uint8_t receive_data
                , uint8_t data_len, uint8_t *data)
{
    if (!(spi->flags & (SF_SOFTWARE|SF_HARDWARE)))
        // Not yet initialized
        return;

    if (CONFIG_HAVE_GPIO_BITBANGING && spi->flags & SF_SOFTWARE)
        spi_software_prepare(spi->spi_software);
    else
        spi_prepare(spi->spi_config);

    if (spi->flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, 0);

    if (CONFIG_HAVE_GPIO_BITBANGING && spi->flags & SF_SOFTWARE)
        spi_software_transfer(spi->spi_software, receive_data, data_len, data);
    else
        spi_transfer(spi->spi_config, receive_data, data_len, data);

    if (spi->flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, 1);
}

void
command_spi_transfer(uint32_t *args)
{
    uint8_t oid = args[0];
    struct spidev_s *spi = spidev_oid_lookup(oid);
    uint8_t data_len = args[1];
    uint8_t *data = (void*)(size_t)args[2];
    spidev_transfer(spi, 1, data_len, data);
    sendf("spi_transfer_response oid=%c response=%*s", oid, data_len, data);
}
DECL_COMMAND(command_spi_transfer, "spi_transfer oid=%c data=%*s");

void
command_spi_send(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    uint8_t data_len = args[1];
    uint8_t *data = (void*)(size_t)args[2];
    spidev_transfer(spi, 0, data_len, data);
}
DECL_COMMAND(command_spi_send, "spi_send oid=%c data=%*s");


/****************************************************************
 * Shutdown handling
 ****************************************************************/

struct spidev_shutdown_s {
    struct spidev_s *spi;
    uint8_t shutdown_msg_len;
    uint8_t shutdown_msg[];
};

void
command_config_spi_shutdown(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[1]);
    uint8_t shutdown_msg_len = args[2];
    struct spidev_shutdown_s *sd = oid_alloc(
        args[0], command_config_spi_shutdown, sizeof(*sd) + shutdown_msg_len);
    sd->spi = spi;
    sd->shutdown_msg_len = shutdown_msg_len;
    uint8_t *shutdown_msg = (void*)(size_t)args[3];
    memcpy(sd->shutdown_msg, shutdown_msg, shutdown_msg_len);
}
DECL_COMMAND(command_config_spi_shutdown,
             "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s");

void
spidev_shutdown(void)
{
    // Cancel any transmissions that may be in progress
    uint8_t oid;
    struct spidev_s *spi;
    foreach_oid(oid, spi, command_config_spi) {
        if (spi->flags & SF_HAVE_PIN)
            gpio_out_write(spi->pin, 1);
    }

    // Send shutdown messages
    struct spidev_shutdown_s *sd;
    foreach_oid(oid, sd, command_config_spi_shutdown) {
        spidev_transfer(sd->spi, 0, sd->shutdown_msg_len, sd->shutdown_msg);
    }
}
DECL_SHUTDOWN(spidev_shutdown);
#endif
