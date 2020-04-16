#!/usr/bin/env python2
# Script to parse a serial port data dump
#
# Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, logging, struct, math
from collections import deque, namedtuple
import msgproto

steppers = {}
last_clock = 0
clock_base = 0

Move = namedtuple('Move',
    ['interval', 'count', 'add', 'pos', 'ticks', 'startclock'])

def find_pos(stepper, clock):
    # find the move whith comprises clock
#    if clock != 1335904042:
#        return 0
    q = stepper['queue']
    startmove = None
    for m in reversed(q):
        print("start %d clock %d" % (m.startclock, clock))
        if (m.startclock > clock and m.startclock - clock < (1 << 31) or
          m.startclock <= clock and clock - m.startclock >= (1 << 31)):
            continue
        startmove = m
        pos = m.pos
        break

    if startmove is None:
        return -1
    print("startmove", startmove)
    # find step withing move
    # XXX wrap
    if clock < m.startclock or clock >= m.startclock + m.ticks:
            return -2
    i = m.interval
    a = m.add
    x = clock - m.startclock
    if a == 0:
        c = x / i
        return m.pos + c
    p = (2. * i / a) + 1
    q = -(2. / a) * x
    c1 = -(p / 2.) + math.sqrt(p ** 2 + q)
    c2 = -(p / 2.) - math.sqrt(p ** 2 + q)
    print("c1 %f c2 %f" % (c1, c2))

    return pos

def process(header, m):
    global steppers, last_clock, clock_base
    l = m.split(' ')
    cmd = l[0]
    if cmd not in ['fpga_reset_step_clock', 'fpga_set_next_step_dir',
                   'fpga_dro_data', 'fpga_queue_step', 'clock']:
        return None
    annotation = ""
    params = {}
    if len(l) > 1:
        for r in l[1:]:
            r = r.split('=')
            params[r[0]] = r[1]
    if cmd == 'clock':
        clock = int(params['clock'])
        if clock < last_clock:
            clock_base += 2**32
        last_clock = clock
        last_time = header['stime']
        annotation = "base %d" % (clock_base)
        return annotation
    oid = params['oid']
    if cmd == 'fpga_reset_step_clock':
        # fpga_reset_step_clock oid=9 clock=0
        steppers[oid] = {
            'next_clock': 0,
            'pos': 0,
            'queue': deque(),
        }
    elif cmd == 'fpga_set_next_step_dir':
        # fpga_set_next_step_dir oid=11 dir=0
        steppers[oid]['dir'] = params['dir']
    elif cmd == 'fpga_queue_step':
        # fpga_queue_step oid=11 interval=34902521 count=1 add=0
        s = steppers[oid]
        interval = int(params['interval'])
        count = int(params['count'])
        add = int(params['add'])
        ticks = interval * count + add * (count / 2) * ( count + 1)
        annotation = "at %d pos %d" % (s['next_clock'] % 2**32, s['pos'])
        move = Move(interval, count, add, s['pos'], ticks,
            s['next_clock'] % 2**32)
        q = steppers[oid]['queue'];
        q.append(move)
        if (len(q) > 2000):
            q.popleft()
        steppers[oid]['next_clock'] += ticks
        if (steppers[oid]['dir']):
            steppers[oid]['pos'] += count
        else:
            steppers[oid]['pos'] -= count
    elif cmd == 'fpga_dro_data':
        # fpga_dro_data oid=6 clock=1273905427 data=6255616 bits=24
        clock = int(params['clock'])
        data = int(params['data'])
        bits = int(params['bits'])
        if bits != 24:
            return "<bad length>"
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
        annotation = "val %.3f" % (val)
        # find pos from all steppers
        for (oid, s) in steppers.items():
            pos = find_pos(s, clock)
            print("oid %s pos %d" % (oid, pos))
    return annotation

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

def format_header(header):
    if header['type']:
        return "--> %f " % (header['stime'])
    else:
        return "<-- %f " % (header['stime'])

# check for a full message frame
def check_packet(data):
    hlen = 18
    if len(data) < hlen:
        return 0, None, None
    mlen = ord(data[17])
    if len(data) < hlen + mlen:
        return 0, None, None
    header = dict(zip(('stime', 'rtime', 'type', 'msg_len'),
                  struct.unpack("ddBB", data[:18])))
    return hlen + mlen, data[hlen:hlen+mlen], header

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
                phead = format_header(header)
                for m in msgs[1:]:
                    a = process(header, m)
                    if a is None:
                        continue
                    sys.stdout.write(phead + m + ' ' + a + '\n')
                msg = msg[ml:]

if __name__ == '__main__':
    main()
