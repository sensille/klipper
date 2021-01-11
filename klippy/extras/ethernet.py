# Support for ethernet data acquisition output from FPGA
#
# Copyright (C) 2020  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class Ethernet:
    def __init__(self, config):
        self.printer = config.get_printer()
        name = config.get_name().split()[1]
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('channel'))
        self.pin = pin_params['pin']
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
	self.phy = config.getint('phy', 1)
	self.src_mac = self.parse_mac(config.get('src_mac'))
	self.dst_mac = self.parse_mac(config.get('dst_mac'))
    def build_config(self):
        macs1 = self.src_mac >> 16
        macs2 = (self.src_mac & 0xffff) << 16 | self.dst_mac >> 32
        macs3 = self.dst_mac & 0xffffffff
        self.mcu.add_config_cmd(
            "config_ether oid=%d channel=%s phy=%u macs1=%u macs2=%u macs3=%u"
            % (self.oid, self.pin, self.phy, macs1, macs2, macs3))
        self.mcu.add_config_cmd("ether_set_state oid=%d state=%u"
            % (self.oid, 2))
    def parse_mac(self, mac):
        res = 0
        bytes = mac.split(':')
        if (len(bytes) != 6):
            raise self.printer.config_error("Invalid mac '%s'" % (mac,))
        for b in bytes:
            res = res << 8
            v = int(b, 16)
            if (v < 0 or v > 255):
                raise self.printer.config_error("Invalid mac '%s'" % (mac,))
            res = res | v
        return res

def load_config_prefix(config):
    return Ethernet(config)
