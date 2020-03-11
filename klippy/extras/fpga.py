# Interface to FPGA behind micro-controller
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, zlib, logging, math
import serialhdl, pins, chelper, mcu, clocksync

class error(Exception):
    pass

class FPGA:
    error = error
    def __init__(self, config):
        self._printer = printer = config.get_printer()
        self._reactor = printer.get_reactor()
        self._name = config.get_name()
        if self._name.startswith('fpga '):
            self._name = self._name[5:]
        printer.register_event_handler("klippy:connect", self._connect)
        printer.register_event_handler("klippy:mcu_identify",
                                             self._mcu_identify)
        printer.register_event_handler("klippy:shutdown", self._shutdown)
        printer.register_event_handler("klippy:disconnect", self._disconnect)
        mcu_name = config.get('mcu', 'mcu')
        self._mcu = printer.lookup_object(mcu_name)
        self._emergency_stop_cmd = None
        # Config building
        printer.lookup_object('pins').register_chip(self._name, self)
        self._oid_count = 0
        self._config_callbacks = []
        self._init_cmds = []
        self._config_cmds = []
        self._pin_map = config.get('pin_map', None)
        self._custom = config.get('custom', '')
        # Move command queuing
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._max_stepper_error = config.getfloat(
            'max_stepper_error', 0.000025, minval=0.)
        self._stepqueues = []
        self._steppersync = None
        self._fid = self._mcu.create_fid()
        printer.add_object('mcu', self)
    # Serial callbacks
    # Connection phase
    def _check_restart(self, reason):
        start_reason = self._printer.get_start_args().get("start_reason")
        if start_reason == 'firmware_restart':
            return
        logging.info("Attempting automated FPGA '%s' restart: %s",
                     self._name, reason)
        self._printer.request_exit('firmware_restart')
        self._reactor.pause(self._reactor.monotonic() + 2.000)
        raise error("Attempt FPGA '%s' restart failed" % (self._name,))
    def _add_custom(self):
        for line in self._custom.split('\n'):
            line = line.strip()
            cpos = line.find('#')
            if cpos >= 0:
                line = line[:cpos].strip()
            if not line:
                continue
            self.add_config_cmd(line)
    def _send_config(self, prev_crc):
        # Build config commands
        for cb in self._config_callbacks:
            cb()
        self._add_custom()
        self._config_cmds.insert(0, "allocate_oids count=%d" % (
            self._oid_count,))
        # Resolve pin names
        fpga_type = self._serial.get_msgparser().get_constant('FPGA')
        ppins = self._printer.lookup_object('pins')
        pin_resolver = ppins.get_pin_resolver(self._name)
        if self._pin_map is not None:
            pin_resolver.add_pin_mapping(fpga_type, self._pin_map)
        for i, cmd in enumerate(self._config_cmds):
            self._config_cmds[i] = pin_resolver.update_command(cmd)
        for i, cmd in enumerate(self._init_cmds):
            self._init_cmds[i] = pin_resolver.update_command(cmd)
        # Calculate config CRC
        config_crc = zlib.crc32('\n'.join(self._config_cmds)) & 0xffffffff
        self.add_config_cmd("finalize_config crc=%d" % (config_crc,))
        # Transmit config messages (if needed)
        if prev_crc is None:
            logging.info("Sending FPGA '%s' printer configuration...",
                         self._name)
            for c in self._config_cmds:
                self._serial.send(c)
        elif config_crc != prev_crc:
            self._check_restart("CRC mismatch")
            raise error("FPGA '%s' CRC does not match config" % (self._name,))
        # Transmit init messages
        for c in self._init_cmds:
            self._serial.send(c)
    def _send_get_config(self):
        get_config_cmd = self.lookup_query_command(
            "get_config",
            "config is_config=%c crc=%u move_count=%hu is_shutdown=%c")
        if self.is_fileoutput():
            return { 'is_config': 0, 'move_count': 500, 'crc': 0 }
        config_params = get_config_cmd.send()
        if self._is_shutdown:
            raise error("FPGA '%s' error during config: %s" % (
                self._name, self._shutdown_msg))
        if config_params['is_shutdown']:
            raise error("Can not update FPGA '%s' config as it is shutdown" % (
                self._name,))
        return config_params
    def _log_info(self):
        msgparser = self._serial.get_msgparser()
        log_info = [
            "Loaded FPGA '%s' %d commands (%s / %s)" % (
                self._name, len(msgparser.messages_by_id),
                msgparser.version, msgparser.build_versions),
            "FPGA '%s' config: %s" % (self._name, " ".join(
                ["%s=%s" % (k, v) for k, v in self.get_constants().items()]))]
        return "\n".join(log_info)
    def _connect(self):
        config_params = self._send_get_config()
        if not config_params['is_config']:
            if self._restart_method == 'rpi_usb':
                # Only configure fpga after usb power reset
                self._check_restart("full reset before config")
            # Not configured - send config and issue get_config again
            self._send_config(None)
            config_params = self._send_get_config()
            if not config_params['is_config'] and not self.is_fileoutput():
                raise error("Unable to configure FPGA '%s'" % (self._name,))
        else:
            start_reason = self._printer.get_start_args().get("start_reason")
            if start_reason == 'firmware_restart':
                raise error("Failed automated reset of FPGA '%s'" % (
                    self._name,))
            # Already configured - send init commands
            self._send_config(config_params['crc'])
        # Setup steppersync with the move_count returned by get_config
        move_count = config_params['move_count']
        self._steppersync = self._ffi_lib.steppersync_alloc(
            self._serial.serialqueue, self._stepqueues, len(self._stepqueues),
            move_count)
        self._ffi_lib.steppersync_set_time(
            self._steppersync, 0., self._fpga_freq)
        # Log config information
        move_msg = "Configured FPGA '%s' (%d moves)" % (self._name, move_count)
        logging.info(move_msg)
        log_info = self._log_info() + "\n" + move_msg
        self._printer.set_rollover_info(self._name, log_info, log=False)
    def _mcu_identify(self):
        # our mcu is already identified. Configure fpga here and get version
        mcu = self._mcu
        cmd = mcu.lookup_query_command(
            "config_fpga fid=%c clk_pin=%u miso_pin=%u mosi_pin=%u cs_pin=%u "
            "program_pin=%u init_pin=%u done_pin=%u di_pin=%u",
            "fpga_init_done fid=%c", async=True, timeout=10, retry_time=100)
        cmd.send([self._fid, 'PB3', 'PB4', 'PB5', 'PB0', 'PC13', 'PC14',
            'PC15', 'PB1'])
        logging.info(self._log_info())
        ppins = self._printer.lookup_object('pins')
        pin_resolver = ppins.get_pin_resolver(self._name)
        for cname, value in self.get_constants().items():
            if cname.startswith("RESERVE_PINS_"):
                for pin in value.split(','):
                    pin_resolver.reserve_pin(pin, cname[13:])
        self._fpga_freq = self.get_constant_float('CLOCK_FREQ')
        self._emergency_stop_cmd = self.lookup_command("emergency_stop")
        self._reset_cmd = self.try_lookup_command("reset")
        self._config_reset_cmd = self.try_lookup_command("config_reset")
        ext_only = self._reset_cmd is None and self._config_reset_cmd is None
        mbaud = self._serial.get_msgparser().get_constant('SERIAL_BAUD', None)
        if self._restart_method is None and mbaud is None and not ext_only:
            self._restart_method = 'command'
    # Config creation helpers
    def setup_pin(self, pin_type, pin_params):
        pcs = {'endstop': FPGA_endstop,
               'digital_out': FPGA_digital_out, 'pwm': FPGA_pwm, 'adc': FPGA_adc}
        if pin_type not in pcs:
            raise pins.error("pin type %s not supported on fpga" % (pin_type,))
        return pcs[pin_type](self, pin_params)
    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False):
        if is_init:
            self._init_cmds.append(cmd)
        else:
            self._config_cmds.append(cmd)
    def get_query_slot(self, oid):
        slot = self.seconds_to_clock(oid * .01)
        t = int(self.estimated_print_time(self._reactor.monotonic()) + 1.5)
        return self.print_time_to_clock(t) + slot
    def register_stepqueue(self, stepqueue):
        self._stepqueues.append(stepqueue)
    def seconds_to_clock(self, time):
        return int(time * self._fpga_freq)
    def get_max_stepper_error(self):
        return self._max_stepper_error
    # Wrapper functions
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def register_response(self, cb, msg, oid=None):
        self._serial.register_response(cb, msg, oid)
    def alloc_command_queue(self):
        return self._serial.alloc_command_queue()
    def lookup_command(self, msgformat, cq=None):
        return CommandWrapper(self._serial, msgformat, cq)
    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, async=False):
        return CommandQueryWrapper(self._serial, msgformat, respformat, oid,
                                   cq, async)
    def try_lookup_command(self, msgformat):
        try:
            return self.lookup_command(msgformat)
        except self._serial.get_msgparser().error as e:
            return None
    def lookup_command_id(self, msgformat):
        return self._serial.get_msgparser().lookup_command(msgformat).msgid
    def get_enumerations(self):
        return self._serial.get_msgparser().get_enumerations()
    def get_constants(self):
        return self._serial.get_msgparser().get_constants()
    def get_constant_float(self, name):
        return self._serial.get_msgparser().get_constant_float(name)
    def print_time_to_clock(self, print_time):
        return self._clocksync.print_time_to_clock(print_time)
    def clock_to_print_time(self, clock):
        return self._clocksync.clock_to_print_time(clock)
    def estimated_print_time(self, eventtime):
        return self._clocksync.estimated_print_time(eventtime)
    def get_adjusted_freq(self):
        return self._clocksync.get_adjusted_freq()
    def clock32_to_clock64(self, clock32):
        return self._clocksync.clock32_to_clock64(clock32)
    # Restarts
    def _disconnect(self):
        self._serial.disconnect()
        if self._steppersync is not None:
            self._ffi_lib.steppersync_free(self._steppersync)
            self._steppersync = None
    def _shutdown(self, force=False):
        if (self._emergency_stop_cmd is None
            or (self._is_shutdown and not force)):
            return
        self._emergency_stop_cmd.send()
    def _restart_arduino(self):
        logging.info("Attempting FPGA '%s' reset", self._name)
        self._disconnect()
        serialhdl.arduino_reset(self._serialport, self._reactor)
    def _restart_via_command(self):
        if ((self._reset_cmd is None and self._config_reset_cmd is None)
            or not self._clocksync.is_active()):
            logging.info("Unable to issue reset command on FPGA '%s'",
                         self._name)
            return
        if self._reset_cmd is None:
            # Attempt reset via config_reset command
            logging.info("Attempting FPGA '%s' config_reset command", self._name)
            self._is_shutdown = True
            self._shutdown(force=True)
            self._reactor.pause(self._reactor.monotonic() + 0.015)
            self._config_reset_cmd.send()
        else:
            # Attempt reset via reset command
            logging.info("Attempting FPGA '%s' reset command", self._name)
            self._reset_cmd.send()
        self._reactor.pause(self._reactor.monotonic() + 0.015)
        self._disconnect()
    def _restart_rpi_usb(self):
        logging.info("Attempting FPGA '%s' reset via rpi usb power", self._name)
        self._disconnect()
        chelper.run_hub_ctrl(0)
        self._reactor.pause(self._reactor.monotonic() + 2.)
        chelper.run_hub_ctrl(1)
    def microcontroller_restart(self):
        if self._restart_method == 'rpi_usb':
            self._restart_rpi_usb()
        elif self._restart_method == 'command':
            self._restart_via_command()
        else:
            self._restart_arduino()
    # Misc external commands
    def is_fileoutput(self):
        return self._printer.get_start_args().get('debugoutput') is not None
    def is_shutdown(self):
        return self._is_shutdown
    def flush_moves(self, print_time):
        if self._steppersync is None:
            return
        clock = self.print_time_to_clock(print_time)
        if clock < 0:
            return
        ret = self._ffi_lib.steppersync_flush(self._steppersync, clock)
        if ret:
            raise error("Internal error in FPGA '%s' stepcompress" % (
                self._name,))
    def check_active(self, print_time, eventtime):
        if self._steppersync is None:
            return
        offset, freq = self._clocksync.calibrate_clock(print_time, eventtime)
        self._ffi_lib.steppersync_set_time(self._steppersync, offset, freq)
        if (self._clocksync.is_active() or self.is_fileoutput()
            or self._is_timeout):
            return
        self._is_timeout = True
        logging.info("Timeout with FPGA '%s' (eventtime=%f)",
                     self._name, eventtime)
        self._printer.invoke_shutdown("Lost communication with FPGA '%s'" % (
            self._name,))
    def __del__(self):
        self._disconnect()

def load_config_prefix(config):
    return FPGA(config)
