# Support for ethernet data acquisition output from FPGA
#
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class Signal:
    def __init__(self, config):
        self.printer = config.get_printer()
        name = config.get_name().split()[1]
        mcu_name = config.get('mcu')
        self.mcu = self.printer.lookup_object('mcu ' + mcu_name)
        self.mcu.register_config_callback(self.build_config)
        self.enable = config.getboolean('enable', False)
        self.mask = int(config.get('mask'), 16)
    def build_config(self):
        self.mcu.add_config_cmd("config_signal enable=%u mask=%u"
            % (self.enable, self.mask))

def load_config_prefix(config):
    return Signal(config)
