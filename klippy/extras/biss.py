# Support for BiSS-C of icHaus sensors
#
# Copyright (C) 2021  Arne Jansen <arne@die-jansens.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import time

class BISS:
    def __init__(self, config):
        self.printer = config.get_printer()
        name = config.get_name().split()[1]
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('channel'))
        self.pin = pin_params['pin']
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.timeout = config.getfloat('timeout', .00004, above=0.0000125,
		below=0.000050)
        self.freq = config.getfloat('freq', 500000, above=80, below=10000000)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
    def build_config(self):
        self.mcu.add_config_cmd("config_biss oid=%d biss=%s freq=%u timeout=%u"
            % (self.oid, self.pin,
            self.mcu.seconds_to_clock(1. / self.freq / 2),
           self.mcu.seconds_to_clock(self.timeout)))
    def _handle_connect(self):
        bits = 25
        logging.info("biss connect, own oid is %d", self.oid)
        self.cmd_frame = self. mcu.lookup_query_command(
            "biss_frame oid=%c cdm=%c bits=%c",
            "biss_data oid=%c data=%*s", oid=self.oid, is_async=True)
        #for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 66, 67,
        #          118, 119, 120, 121, 122, 123, 124, 125, 126, 127]:
        #    v = self.read_register(0, i, bits)
        #    print("read %02x from %02x" % (v, i))
        for i in [118, 119]:
            v = self.read_register(0, i, bits)
            print("read %02x from %02x" % (v, i))
        v = self.read_register(0, 4, bits)
        self.write_register(0, 5, 0x03, bits)  # driver 3MHz, 20mA
        self.write_register(0, 4, v | 0x40, bits)  # deactivate pullup on NERR
        self.write_register(0, 6, 0x82, bits)      # resolution 4096
        self.write_register(0, 8, 0x10 | 0x00, bits)  # min edge 2MHz, 0.17deg
        v = self.read_register(0, 10, bits)
        self.write_register(0, 10, v | 0x08, bits) # disable GAIN output
        self.write_register(0, 0, 0xc0, bits)      # GAING x10
        self.write_register(0, 1, 0x92, bits)      # ENAC=1, GCC=0
        v = self.read_register(0, 2, bits)
        v &= 0x80;
        self.write_register(0, 2, v | 0x0, bits)      # VOSS=1
        self.write_register(0, 3, 0x40, bits)      # VOSC=1
        for _ in range(1, 4):
            for i in [118, 119]:
                v = self.read_register(0, i, bits)
                print("read %02x from %02x" % (v, i))
            for _ in range(1, 0):
                frame = self.transfer_frame(0, bits)
                fdata = frame['data']
                v = (fdata[0] << 16) + (fdata[1] << 8) + fdata[2]
                print("val %d nE %d nW %d" %((v >> 7) & 4095,
                    (v >> 6) & 1, (v >> 5) & 1))
                logging.info(frame)
            time.sleep(1)
    def transfer_frame(self, cdm, bits):
        rsp = self.cmd_frame.send([self.oid, 1 - cdm, bits])
        data = bytearray(rsp['data'])
        if (data[0] & 0xf0) != 0xd0:
            raise self.mcu.get_printer().config_error(
                "invalid biss frame (start of frame)")
        frame = {}
        frame['cds'] = (data[0] >> 3) & 1
        frame['data'] = data
        return frame
    def transfer_ctrl_frame(self, frame, bits):
        rsp = []
        for cdm in frame:
            f = self.transfer_frame(cdm, bits)
            rsp.append(f['cds'])
        return rsp
    def _rw_register(self, id, addr, data, do_write, bits):
        frame = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        crc_seq = [1, id >> 2, (id >> 1) & 1, id & 1]
        for i in range (0, 7):
            crc_seq.append((addr >> 6) & 1)
            addr = addr << 1
        frame.extend(crc_seq)
        crc = self.calc_crc4(crc_seq)
        for i in range (0, 4):
            frame.append((crc >> 3) & 1)
            crc = crc << 1
        if (do_write):
            dout = data
            crc_seq = []
            frame.extend([0, 1, 1]) # write
            for i in range (0, 8):
                crc_seq.append((dout >> 7) & 1)
                dout = dout << 1
            frame.extend(crc_seq)
            crc = self.calc_crc4(crc_seq)
            for i in range (0, 4):
                frame.append((crc >> 3) & 1)
                crc = crc << 1
        else:
            frame.extend([1, 0, 1]) # read
            frame.extend([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        frame.append(0) # P
        frame.append(0) # one more bit to receive the stop bit from slave
        rsp = self.transfer_ctrl_frame(frame, bits)
        # check frame
        exp = [0] * 15
        exp.append(1)   # id 0
        exp.extend([0] * 15)
        if (do_write):
            exp.extend([0, 1, 1])
        else:
            exp.extend([1, 0, 1])
        for i in range(0, 34):
            if rsp[i] != exp[i]:
                raise self.mcu.get_printer().config_error(
                    "invalid ctrl response (start of frame)")
        rdata = 0
        for i in range(0, 8):
            rdata = rdata << 1
            rdata = rdata | rsp[34 + i]
        crc = 0
        for i in range(0, 4):
            crc = crc << 1
            crc = crc | rsp[42 + i]
        if crc != self.calc_crc4(rsp[34:42]):
                raise self.mcu.get_printer().config_error(
                    "invalid crc response (read)")
        if do_write:
            if rdata != data:
                raise self.mcu.get_printer().config_error(
                    "received data don't match (write)")
        return rdata
    def read_register(self, id, addr, bits):
        return self._rw_register(id, addr, 0, 0, bits)
    def write_register(self, id, addr, data, bits):
        return self._rw_register(id, addr, data, 1, bits)
    def calc_crc4(self, crc_seq):
        crc = 0
        # x^4 + x^1 + x^0
        for b in crc_seq:
            if b != ((crc >> 3) & 1):
                crc = ((crc << 1) & 0x0f) ^ 3
            else:
                crc = ((crc << 1) & 0x0f)
        return crc ^ 0x0f

def load_config_prefix(config):
    return BISS(config)
