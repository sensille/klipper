# Support for DRO of dial indicators/digital calipers
#
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class DRO:
    def __init__(self, config):
        self.printer = config.get_printer()
        name = config.get_name().split()[1]
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('channel'))
        self.pin = pin_params['pin']
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.timeout = config.getfloat('timeout', .001, above=0., below=1.)
        self.daq = config.getboolean('daq', False)
    def build_config(self):
        self.mcu.add_config_cmd("config_dro oid=%d dro=%s timeout=%u daq=%u"
            % (self.oid, self.pin, self.mcu.seconds_to_clock(self.timeout),
               self.daq))
        self.mcu.register_response(self._handle_dro_data, "dro_data", self.oid)
    def _handle_dro_data(self, params):
        clock = params['clock']
        data = params['data']
        bits = params['bits']
        if bits != 24:
            logging.warn("bad dro data length received: %d", bits)
            return
        flipped = 0
        for i in range(bits):
            flipped <<= 1
            flipped |= data & 1
            data >>= 1
        data = flipped
        sign = data & (1 << 20)
        val = (data & ((1 << 20) - 1)) / 1000.
        if sign:
            val = -val
        logging.info("dro_data: clock %d value %.3f", clock, val)

def load_config_prefix(config):
    return DRO(config)
