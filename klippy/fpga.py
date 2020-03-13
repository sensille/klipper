# Interface to FPGA behind micro-controller
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, zlib, logging, math
import serialhdl, pins, chelper, mcu, clocksync

# Wrapper around command sending
class CommandWrapper:
    def __init__(self, fpga, mcu, msgformat, cmd_queue=None):
        self._serial = serial
        self._cmd = serial.get_msgparser().lookup_command(msgformat)
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue
    def send(self, data=(), minclock=0, reqclock=0):
        cmd = self._cmd.encode(data)
        self._serial.raw_send(cmd, minclock, reqclock, self._cmd_queue)

class FPGA:
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
    def _log_info(self):
        log_info = [
            "Loaded FPGA '%s'" % (self._name),
            "FPGA '%s' config: %s" % (self._name, " ".join(
                ["%s=%s" % (k, v) for k, v in self._config.items()]))]
        return "\n".join(log_info)
    def _connect(self):
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
        pcs = {'endstop': mcu.MCU_endstop,
               'digital_out': mcu.MCU_digital_out, 'pwm': mcu.MCU_pwm}
        if pin_type not in pcs:
            raise pins.error("pin type %s not supported on fpga" % (pin_type,))
        return pcs[pin_type](self, pin_params)
    def create_oid(self):
        return self._mcu.create_oid()
    def register_config_callback(self, cb):
        self._mcu.register_config_callback(cb)
    def add_config_cmd(self, cmd, is_init=False):
        self._remap_cmd(cmd)
        self._mcu.add_config_cmd(cmd, is_init)
    def get_query_slot(self, oid):
        return self._mcu.get_query_slot(oid)
    def register_stepqueue(self, stepqueue):
        raise "XXX"
        self._stepqueues.append(stepqueue)
    def seconds_to_clock(self, time):
        return self._mcu.seconds_to_clock(time)
    def get_max_stepper_error(self):
        raise "XXX"
        return self._max_stepper_error
    # Wrapper functions
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def register_response(self, cb, msg, oid=None):
        raise "XXX"
        self._serial.register_response(cb, msg, oid)
    def alloc_command_queue(self):
        return self._mcu.alloc_command_queue()
    def lookup_command(self, msgformat, cq=None):
        print("lookup ", msgformat)
        return self._mcu.lookup_command(msgformat, cq)
        #return mcu.CommandWrapper(self._serial, msgformat, cq)
    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, async=False):
        raise "XXX"
        return CommandQueryWrapper(self._serial, msgformat, respformat, oid,
                                   cq, async)
    def try_lookup_command(self, msgformat):
        try:
            return self.lookup_command(msgformat)
        except self._serial.get_msgparser().error as e:
            return None
    def lookup_command_id(self, msgformat):
        raise "XXX"
        return self._serial.get_msgparser().lookup_command(msgformat).msgid
    def get_enumerations(self):
        raise "XXX"
        return self._serial.get_msgparser().get_enumerations()
    def get_constants(self):
        raise "XXX"
        return self._serial.get_msgparser().get_constants()
    def get_constant_float(self, name):
        raise "XXX"
        return self._serial.get_msgparser().get_constant_float(name)
    def print_time_to_clock(self, print_time):
        return self._mcu.print_time_to_clock(print_time)
    def clock_to_print_time(self, clock):
        return self._clocksync.clock_to_print_time(clock)
    def estimated_print_time(self, eventtime):
        return self._mcu.estimated_print_time(eventtime)
    def get_adjusted_freq(self):
        raise "XXX"
        return self._clocksync.get_adjusted_freq()
    def clock32_to_clock64(self, clock32):
        raise "XXX"
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
        raise "XXX"
        return self._is_shutdown
    def flush_moves(self, print_time):
        raise "XXX"
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
        raise "XXX"
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

def add_printer_objects(config):
    printer = config.get_printer()
    for s in config.get_prefix_sections('fpga '):
        fpga = FPGA(s)
        printer.add_object('mcu ' + fpga.get_name(), fpga)
