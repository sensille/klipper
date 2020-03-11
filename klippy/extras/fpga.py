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
        self._flash_clk = config.get('flash_clk')
        self._flash_miso = config.get('flash_miso')
        self._flash_mosi = config.get('flash_mosi')
        self._flash_cs = config.get('flash_cs')
        self._program = config.get('program')
        self._init = config.get('init')
        self._done = config.get('done')
        self._di = config.get('di')
        self._usart_bus = config.get('usart')
        self._usart_rate = config.getint('baud')
        self._timesync = config.get('timesync')
        self._serr = config.get('serr')
        self._sreset = config.get('sreset')
        self._mcu = printer.lookup_object(mcu_name)
        self._fid = self._mcu.create_fid()
        self._emergency_stop_cmd = None
        # Config building
        printer.lookup_object('pins').register_chip(self._name, self)
        print("##### register chip %s" % (self._name))
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
        printer.add_object('mcu ' + self._name, self)
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
    def _send_config(self, prev_crc):
        # Build config commands
        for cb in self._config_callbacks:
            cb()
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
        logging.info("Sending FPGA '%s' printer configuration...",
                     self._name)
        for c in self._config_cmds:
            self._serial.send(c)
        # Transmit init messages
        for c in self._init_cmds:
            self._serial.send(c)
    def _log_info(self):
        log_info = [
            "Loaded FPGA '%s'" % (self._name),
            "FPGA '%s' config: %s" % (self._name, " ".join(
                ["%s=%s" % (k, v) for k, v in self._config.items()]))]
        return "\n".join(log_info)
    def _connect(self):
        #self._send_config(None)
        # Setup steppersync with the move_count returned by get_config
        move_count = self._config['move_cnt']
        #self._steppersync = self._ffi_lib.steppersync_alloc(
        #    self._serial.serialqueue, self._stepqueues, len(self._stepqueues),
        #    move_count)
        #self._ffi_lib.steppersync_set_time(
        #    self._steppersync, 0., self._mcu_freq)
        # Log config information
        move_msg = "Configured FPGA '%s' (%d moves)" % (self._name, move_count)
        logging.info(move_msg)
    def _mcu_identify(self):
        # our mcu is already identified. Configure fpga here and get version
        mcu = self._mcu

        cmd = mcu.lookup_query_command(
            "config_fpga fid=%c clk_pin=%u miso_pin=%u mosi_pin=%u cs_pin=%u "
            "program_pin=%u init_pin=%u done_pin=%u di_pin=%u",
            "fpga_init_done fid=%c", async=True, timeout=10, retry_time=100)
        rsp = cmd.send([self._fid, self._flash_clk, self._flash_miso,
                  self._flash_mosi, self._flash_cs, self._program, self._init,
                  self._done, self._di])
        if rsp['fid'] != self._fid:
            raise self._printer.config_error(
                "invalid response during fpga identification")

        cmd = mcu.lookup_query_command(
            "fpga_setup fid=%c usart_bus=%u rate=%u timesync_pin=%u "
            "serr_pin=%u sreset_pin=%u",
            "fpga_config fid=%c version=%u gpio=%c pwm=%c stepper=%c "
            "endstop=%c uart=%c move_cnt=%u", async=True)
        rsp = cmd.send([self._fid, self._usart_bus, self._usart_rate,
                  self._timesync, self._serr, self._sreset])
        if rsp['fid'] != self._fid or rsp['version'] != 66:
            raise self._printer.config_error(
                "invalid response during fpga identification")
        del rsp['fid']
        self._config = {k: rsp[k] for k in rsp if not k.startswith('#')}
        logging.info(self._log_info())
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
    def microcontroller_restart(self):
        pass
    # Misc external commands
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
