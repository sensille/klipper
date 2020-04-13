#!/usr/bin/env python2
# Script to parse a serial port data dump
#
# Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, logging, struct
import msgproto

def read_dictionary(filename):
    dfile = open(filename, 'rb')
    dictionary = dfile.read()
    dfile.close()
    return dictionary

#    double sent_time;                  8
#    double receive_time;               8
#    uint8_t type;   // send/receive    1
#    uint8_t msg_len;                   1
#    uint8_t msg[MESSAGE_MAX];

def format_header(data):
    (stime, rtime, type, msg_len) = struct.unpack("ddBB", data)
    if type:
        return "--> %f " % (stime)
    else:
        return "<-- %f " % (stime)

# check for a full message frame
def check_packet(data):
    hlen = 18
    if len(data) < hlen:
        return 0, None, None
    mlen = ord(data[17])
    if len(data) < hlen + mlen:
        return 0, None, None
    return hlen + mlen, data[hlen:hlen+mlen], format_header(data[:18])

def main():
    dict_filename, data_filename = sys.argv[1:]

    dictionary = read_dictionary(dict_filename)

    mp = msgproto.MessageParser()
    mp.process_identify(dictionary, decompress=False)

    f = open(data_filename, 'rb')
    fd = f.fileno()
    data = ""
    while 1:
        newdata = os.read(fd, 4096)
        if not newdata:
            break
        data += newdata
        while 1:
            l, msg, header = check_packet(data)
            if l == 0:
                break
            if l < 0:
                print("Invalid data (framing)")
                exit()
            data = data[l:]
            while 1:
                ml = mp.check_packet(msg)
                if ml == 0:
                    break
                if ml < 0:
                    print("Invalid data (msg)")
                    exit()
                msgs = mp.dump(bytearray(msg[:ml]))
                for m in msgs[1:]:
                    sys.stdout.write(header + m + '\n')
                msg = msg[ml:]

if __name__ == '__main__':
    main()
