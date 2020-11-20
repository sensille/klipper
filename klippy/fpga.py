# Interface to FPGA behind micro-controller
#
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, zlib, logging, math, re
import serialhdl, pins, chelper, mcu, clocksync

class FPGA_tmc_uart:
    def __init__(self, rx_pin_params, tx_pin_params, select_pins_desc, addr):
        self._mcu = rx_pin_params['chip']
        self.rx_pin = rx_pin_params['pin']
        self.addr = addr
        if select_pins_desc is not None:
            raise self._mcu.get_printer().config_error(
                "select_pins not supported for FPGA tmc_uart")
        if tx_pin_params != rx_pin_params:
            raise self._mcu.get_printer().config_error(
                "separate tx pin not supported for FPGA tmc_uart")
        self.oid = self._mcu.create_oid()
        self._registered = False
        self.tmcuart_send_cmd = None
        self.mutex = self._mcu.get_printer().get_reactor().mutex()
        self._mcu.register_config_callback(self.build_config)
    def build_config(self):
        self._mcu.add_config_cmd(
            "config_tmcuart oid=%d rx_pin=%s slave=%d"
            % (self.oid, self.rx_pin, self.addr))
        self.tmcuart_read_cmd = self._mcu.lookup_query_command(
            "tmcuart_read oid=%c register=%c",
            "tmcuart_data oid=%c status=%c data=%u", oid=self.oid,
            is_async=True)
        self.tmcuart_write_cmd = self._mcu.lookup_command(
            "tmcuart_write oid=%c register=%c data=%u")
    def register_instance(self, rx_pin_params, tx_pin_params,
                          select_pins_desc, addr):
        if self._registered:
            raise self._mcu.get_printer().config_error(
                "Only one instance for FPGA uarts allowed")
        self._registered = True
        return 1
    def reg_read(self, instance_id, addr, reg):
        if addr != self.addr:
            raise "XXX move addressing to each command"
        rsp = self.tmcuart_read_cmd.send([self.oid, reg])
        if rsp['status'] != 0:
            raise self._mcu.get_printer().config_error(
                "XXX runtime error: status is %s pin %s" %
                    (rsp['status'], self.rx_pin))
        return rsp['data']
    def reg_write(self, instance_id, addr, reg, val, print_time=None):
        if addr != self.addr:
            raise "XXX move addressing to each command"
        minclock = 0
        if print_time is not None:
            minclock = self._mcu.print_time_to_clock(print_time)
        self.tmcuart_write_cmd.send([self.oid, reg, val], minclock=minclock)

class FPGA:
    _cmds_with_fid = ('config_digital_out')
    _cmdmap = {
        # Value: (fid-needed, pin remapping)
        #        pin remapping: (index, (classes))
        'config_digital_out': (True, ((2, 'channel', ('pwm', 'gpio')),)),
        'schedule_digital_out': (False, ()),
        'config_soft_pwm_out': (True, ((2, 'channel', ('pwm',)),)),
        'schedule_soft_pwm_out': (False, ()),
        'config_tmcuart': (True, ((2, 'channel', ('uart')),)),
        'tmcuart_read': (False, ()),
        'tmcuart_write': (False, ()),
        'tmcuart_data': (False, ()),
        'config_stepper': (True, ((2, 'step', ('step',)),
                                  (3, 'dir',  ('dir',)))),
        'reset_step_clock': (False, ()),
        'queue_step': (False, ()),
        'set_next_step_dir': (False, ()),
        'stepper_get_position': (False, ()),
        'stepper_get_next_step': (False, ()),
        'stepper_position': (False, ()),
        'stepper_next_step': (False, ()),
        'config_endstop': (True, ((2, 'channel', ('endstop')),)),
        'endstop_home': (False, ()),
        'endstop_set_stepper': (False, ()),
        'endstop_query_state': (False, ()),
        'endstop_state': (False, ()),
        'config_dro': (True, ((2, 'channel', ('dro',)),)),
        'dro_data': (False, ()),
        'config_as5311': (True, ((2, 'channel', ('asm',)),)),
        'as5311_data': (False, ()),
    }
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
        self._pin_map = config.get('pin_map', None)
        self._custom = config.get('custom', '')
        # Move command queuing
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._max_stepper_error = config.getfloat(
            'max_stepper_error', 0.000025, minval=0.)
        #self._stepqueues = []
        self._steppersync = None
        printer.add_object('mcu ' + self._name, self)
    def _log_info(self):
        log_info = [
            "Loaded FPGA '%s'" % (self._name),
            "FPGA '%s' config: %s" % (self._name, " ".join(
                ["%s=%s" % (k, v) for k, v in self._config.items()]))]
        return "\n".join(log_info)
    def _connect(self):
        # just make sure the move queue of the fpga is at least as long as
        # that of the mcu
        mcu_move_count = self._mcu.move_count
        fpga_move_count = self._config['move_cnt']
        if fpga_move_count < mcu_move_count:
            raise self._mcu.get_printer().config_error(
                "FPGA move queue is smaller than that of MCU, this won't work")
    def _mcu_identify(self):
        # our mcu is already identified. Configure fpga here and get version
        mcu = self._mcu

        cmd = mcu.lookup_query_command(
            "config_fpga fid=%c clk_pin=%u miso_pin=%u mosi_pin=%u cs_pin=%u "
            "program_pin=%u init_pin=%u done_pin=%u di_pin=%u",
            "fpga_init_done fid=%c", is_async=True, timeout=10, retry_time=100)
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
            "endstop=%c uart=%c sd=%c dro=%c asm=%c move_cnt=%u", is_async=True)
        rsp = cmd.send([self._fid, self._usart_bus, self._usart_rate,
                  self._timesync, self._serr, self._sreset])
        if rsp['fid'] != self._fid or rsp['version'] != 66:
            raise self._printer.config_error(
                "invalid response during fpga identification")
        del rsp['fid']
        self._config = {k: rsp[k] for k in rsp if not k.startswith('#')}
        self._classes = {k: rsp[k] for k in ('gpio', 'pwm', 'endstop', 'uart',
                                             'dro', 'asm')}
        self._classes['step'] = rsp['stepper']
        self._classes['dir'] = rsp['stepper']
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
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        cmd = self._remap_cmd(cmd, True)
        self._mcu.add_config_cmd(cmd, is_init, on_restart)
    def get_query_slot(self, oid):
        return self._mcu.get_query_slot(oid)
    def register_stepqueue(self, stepqueue):
        self._mcu.register_stepqueue(stepqueue)
    def seconds_to_clock(self, time):
        return self._mcu.seconds_to_clock(time)
    def get_max_stepper_error(self):
        return self._max_stepper_error
    # Wrapper functions
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def register_response(self, cb, msg, oid=None):
        msg = self._remap_cmd(msg, False)
        self._mcu.register_response(cb, msg, oid)
    def alloc_command_queue(self):
        return self._mcu.alloc_command_queue()
    def lookup_command(self, msgformat, cq=None):
        msgformat = self._remap_cmd(msgformat, False)
        return self._mcu.lookup_command(msgformat, cq=cq)
    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, is_async=False):
        msgformat = self._remap_cmd(msgformat, False)
        respformat = self._remap_cmd(respformat, False)
        # all queries to fpga are async
        return self._mcu.lookup_query_command(msgformat, respformat, oid=oid,
                                              cq=cq, is_async=True)
    def try_lookup_command(self, msgformat):
        try:
            return self.lookup_command(msgformat)
        except self._serial.get_msgparser().error as e:
            return None
    def lookup_command_id(self, msgformat):
        msgformat = self._remap_cmd(msgformat, False)
        return self._mcu.lookup_command_id(msgformat)
    def print_time_to_clock(self, print_time):
        return self._mcu.print_time_to_clock(print_time)
    def clock_to_print_time(self, clock):
        return self._mcu.clock_to_print_time(clock)
    def estimated_print_time(self, eventtime):
        return self._mcu.estimated_print_time(eventtime)
    def get_adjusted_freq(self):
        return self._mcu.get_adjusted_freq()
    def clock32_to_clock64(self, clock32):
        return self._mcu.clock32_to_clock64(clock32)
    # Restarts
    def _disconnect(self):
        pass
    def _shutdown(self, force=False):
        if (self._emergency_stop_cmd is None
            or (self._is_shutdown and not force)):
            return
        self._emergency_stop_cmd.send()
    def microcontroller_restart(self):
        pass
    # Misc external commands
    def is_fileoutput(self):
        return self._mcu.is_fileoutput()
    def is_shutdown(self):
        return self._mcu.is_shutdown()
    def flush_moves(self, print_time):
        return self._mcu.flush_moves(print_time)
    def check_active(self, print_time, eventtime):
        return self._mcu.check_active(print_time, eventtime)
    def __del__(self):
        self._disconnect()
    def add_config_digital_out(self, oid, pin, value, default_value,
                               max_duration):
        if not pin.startswith('pwm'):
            raise "XXX"
        channel=int(pin[3:]) - 1
        self._mcu.add_config_cmd(
            "fpga_config_digital_out oid=%d fpga_fid=%s channel=%s value=%d "
            "default_value=%d max_duration=%d"
            % (oid, self._fid, channel, value, default_value, max_duration))
    def tmc_uart(self, rx_pin_params, tx_pin_params, select_pins_desc, addr):
        return FPGA_tmc_uart(rx_pin_params, tx_pin_params, select_pins_desc,
                             addr)
    def _remap_cmd(self, cmd, map_values):
        parts = cmd.split()
        (need_fid, remap) = self._cmdmap[parts[0]]
        if map is None:
            return cmd
        for (index, new_name, classes) in remap:
            name, value = parts[index].split('=')
            mapped = value
            if map_values:
                cls, number = self._resolve_pin(value)
                if not cls in classes:
                    raise self._printer.config_error(
                        "pin %s not allowed here" % (value))
                mapped = number
            parts[index] = "%s=%d" % (new_name, mapped)
        if need_fid:
            parts.insert(1, "fid=%d" % (self._fid))
        return "fpga_" + ' '.join(parts)
    def _resolve_pin(self, pin):
        match = re.match('^([a-zA-Z]+)(\d+)$', pin)
        if not match:
                raise self._printer.config_error(
                    "malformed pin name %s" % (pin))
        cls = match.group(1)
        number = int(match.group(2))

        max = self._classes[cls]
        if max is None:
                raise self._printer.config_error(
                    "bad pin class %s" % (cls))
        if number < 1 or number > max:
                raise self._printer.config_error(
                    "pin %s out of range" % (pin))
        return (cls, number - 1)

def add_printer_objects(config):
    printer = config.get_printer()
    for s in config.get_prefix_sections('fpga '):
        fpga = FPGA(s)
        printer.add_object('mcu ' + fpga.get_name(), fpga)
