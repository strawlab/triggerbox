#!/usr/bin/env python
import serial
import numpy as np
import os, sys, time, threading, Queue, collections
import struct
import argparse
import logging

from time_model import get_time_model, TimeFitError

def uint32(b0,b1,b2,b3):
    return (b3 << 24) + (b2 << 16) + (b1 << 8) + b0

def uint16(b0,b1):
    return (b1 << 8) + b0

if sys.platform.startswith('win'):
    time_func = time.clock
else:
    time_func = time.time

QUERY_DT = 1.0 # n seconds between clock info requests

# Measured on 6 September 2013
LOW_VOLTS = 0.0063
HIGH_VOLTS = 4.13
RANGE_VOLTS = HIGH_VOLTS-LOW_VOLTS

class NoValidSerialError(serial.serialutil.SerialException):
    pass

def _setup_serial(device=None):
    port = device
    if port is not None:
        ports = [port]
    else:
        if sys.platform.startswith('linux'):
            ports = ['/dev/ttyUSB0',
                     '/dev/ttyACM0',
                     ]
        else:
            raise RuntimeError('Do not know default serial ports on your '
                               'platform.')
    ser = None
    for port in ports:
        try:
            ser = serial.Serial(port=port,
                                timeout=0.01,
                                baudrate=115200,
                                )
        except serial.serialutil.SerialException:
            continue
        else:
            break
    if ser is None:
        raise NoValidSerialError('Could not find serial port at any of %r'%ports)
    return ser

class SerialThread(threading.Thread):
    def __init__(self,args,device):
        super(SerialThread,self).__init__(name="triggerbox serial thread",)
        self.__args=args
        self._aout_seq = 0
        self.ICR1_AND_PRESCALER = None

        self.device = device

        self.version_check_done = False

        self._last_aout_sequence = None, None, None
        self._log = logging.getLogger("root.serial")

    def _set_ICR1_AND_PRESCALER( self, new_value ):
        icr1, prescaler = new_value
        chars = struct.pack('<H',icr1) # little-endian unsigned short
        assert len(chars)==2
        vals = [ord(c) for c in chars]
        self.ICR1_AND_PRESCALER = ((vals[1] << 8) + vals[0], prescaler)
        assert self.ICR1_AND_PRESCALER==new_value
        if prescaler==8:
            chars += '1'
        elif prescaler==64:
            chars += '2'
        else:
            raise ValueError('unsupported prescaler: %s'%prescaler)
        self.ser.write('T='+chars)

    def _set_AOUT( self, aout0, aout1 ):
        aout0_chars = struct.pack('<H', aout0) # little-endian unsigned short
        assert len(aout0_chars)==2
        aout1_chars = struct.pack('<H', aout1) # little-endian unsigned short
        assert len(aout1_chars)==2
        chars = aout0_chars + aout1_chars + chr(self._aout_seq)
        self.ser.write('O='+chars)
        self._last_aout_sequence = self._aout_seq, aout0, aout1
        self._aout_seq += 1
        self._aout_seq = self._aout_seq % 256

    def run(self):
        self.last_time = time_func() + 0.5 # give half a second to flush buffers
        self._qi = 0
        self._queries = collections.OrderedDict()

        self._vquery_time = time_func()+5.0
        self._version_check_started = False

        self.raw_q, self.time_q, self.outq, self.aout_q = self.__args

        self.ser = _setup_serial(device=self.device)
        self.ser.open()

        buf = ''
        while 1:

            # handle new commands
            while 1:
                try:
                    cmd_tup = self.outq.get_nowait()
                    cmd = cmd_tup[0]
                    if cmd=='icr1_and_prescaler':
                        new_value = cmd_tup[1]
                        self._set_ICR1_AND_PRESCALER( new_value )
                    elif cmd=='stop_pulses_and_reset':
                        self.ser.write('S0')
                    elif cmd=='start_pulses':
                        self.ser.write('S1')
                    elif cmd=='stop_pulses':
                        self.ser.write('S2')
                    elif cmd=='AOut':
                        aout0, aout1 = cmd_tup[1:3]
                        self._set_AOUT( aout0, aout1 )
                except Queue.Empty:
                    break

            # get all pending data
            buf += self.ser.read()

            # handle pending data
            buf = self._h(buf)

            # perform any ongoing tasks
            now = time_func()
            if (now - self.last_time) > QUERY_DT:
                # request sample

                self._queries[ self._qi ] = now

                self.ser.write( 'P'+chr(self._qi) )

                self._qi = (self._qi + 1) % 256
                self.last_time = now

            # version check
            if not self.version_check_done:
                if not self._version_check_started:
                    if now >= self._vquery_time:
                        self.ser.write( 'V?' )
                        self._version_check_started = True
                if (now - self._vquery_time) > 5.0:
                    raise RuntimeError('no version response')

    def _handle_version(self, value, pulsenumber, count):
        assert value==13
        self._vquery_time = time_func()
        self.version_check_done = value
        self._log.info('connected to triggerbox firmware version %d' % value )

    def _handle_returned_timestamp(self, qi, pulsenumber, count):
        now = time_func()

        while len(self._queries) > 50:
            old_qi = self._queries.popitem(last=False)
            self._log.warn('never got return for query %d'%old_qi)

        try:
            send_timestamp = self._queries.pop( qi )
        except KeyError:
            self._log.warn('could not find original data for query %d'%qi)
            return

        max_error = now-send_timestamp
        if max_error > 0.015: # 15 msec cutoff
            self._log.warn(
                'clock sample took %.1f msec. Ignoring value.'%( max_error*1e3))
            return

        ino_time_estimate = (now+send_timestamp)*0.5

        if self.ICR1_AND_PRESCALER is None:
            self._log.warn('No clock measurements until framerate set.')
            return

        icr1, prescaler = self.ICR1_AND_PRESCALER
        frac = float(count)/icr1
        ino_stamp = pulsenumber + frac

        self.time_q.put( (ino_time_estimate, ino_stamp) )

        self.raw_q.put( (send_timestamp, pulsenumber,
                         int(np.round(frac * 255.0)), now) )

    def _handle_returned_aout(self, seq, pulsenumber, count):
        orig_seq, aout0, aout1 = self._last_aout_sequence
        if seq != orig_seq:
            self._log.warn('AOUT confirmation has wrong sequence number.')
            return

        if self.ICR1_AND_PRESCALER is None:
            self._log.warn('No AOUT confirmation until framerate set.')
            return

        icr1, prescaler = self.ICR1_AND_PRESCALER
        frac = float(count)/icr1

        self.aout_q.put( (pulsenumber, int(np.round(frac * 255.0)),
                          aout0, aout1))

    def _h(self,buf):
        result = buf
        if len(buf) >= 3: # header, length, checksum is minimum
            valid = False

            packet_type = buf[0]
            payload_len = ord(buf[1])

            min_valid_packet_size = 3+payload_len  # header (2) + payload + checksum (1)
            if len(buf) >= min_valid_packet_size:
                expected_chksum = ord(buf[2+payload_len])

                check_buf = buf[2:-1]
                bytes = [ord(char) for char in check_buf]
                actual_chksum = sum( bytes ) % 256

                if actual_chksum == expected_chksum:
                    valid = True
                    n_used_chars = len(check_buf)+3
                else:
                    raise RuntimeError('checksum mismatch')

                if packet_type in ('P','V','O'):
                    assert payload_len==7
                    value = bytes[0]
                    e0,e1,e2,e3 = bytes[1:5]
                    t0,t1 = bytes[5:7]

                    pulsenumber = uint32(e0,e1,e2,e3)
                    count = uint16(t0,t1)

                    if packet_type == 'P':
                        self._handle_returned_timestamp(value, pulsenumber, count )
                    elif packet_type == 'V':
                        self._handle_version(value, pulsenumber, count )
                    elif packet_type == 'O':
                        self._handle_returned_aout(value, pulsenumber, count )
                else:
                    raise ValueError('unknown packet type')

            if valid:
                result = buf[n_used_chars:]

        return result

def volts_to_dac(volts):
    v = np.clip(volts,LOW_VOLTS,HIGH_VOLTS)
    v0 = v-LOW_VOLTS
    v0_frac = v0/RANGE_VOLTS
    dac_float = v0_frac*4095
    dac = int(np.round(dac_float))
    return dac

def queue_to_func(q,func):
    while 1:
        raw = q.get()
        func(*raw)

class TriggerboxDevice(threading.Thread):

    def __init__(self, device):
        super(TriggerboxDevice,self).__init__(name="triggerbox device")
        self.daemon = True

        self._connected = False
        self._log = logging.getLogger("trigger.device")

        self.raw_q = Queue.Queue()
        self.time_q = Queue.Queue()
        self.outq = Queue.Queue()
        self.aout_q = Queue.Queue()
        aout_sender_thread = threading.Thread( target=queue_to_func,
                                               args=(self.aout_q,
                                                     self._notify_aout_confirm),
                                               )
        aout_sender_thread.daemon = True
        aout_sender_thread.start()

        self.expected_trigger_rate = np.nan

        self.times = []

        self.ser_thread = SerialThread(args=(
            self.raw_q,self.time_q,self.outq,self.aout_q),
                                       device=device)
        self.ser_thread.daemon = True
        self.ser_thread.start()

        self._log.info('Waiting until serial device confirmed...')
        while True:
            if not self.ser_thread.is_alive():
                raise RuntimeError('serial thread died')
            if self.ser_thread.version_check_done:
                break
            time.sleep(0.1)

        self._connected = True
        self._notify_connected('v:%s' % self.ser_thread.version_check_done, device)

        #we need to talk to the serial device reguarly, so we implement
        #our own scheduler here
        self.start()

    def _clear_data(self):
        self.times = []
        self.time_model = None
        self._notify_clockmodel(np.nan, np.nan)

    def _trim_data(self,_=None):
        keep_n_times = 500

        if len(self.times) > keep_n_times:
            del self.times[-keep_n_times:]
        return True

    def _get_new_data(self, _=None):
        if not self.ser_thread.is_alive():
            self._notify_fatal_error('serial port thread is not alive')
            return

        # get all pending data
        while 1:
            try:
                raw = self.raw_q.get_nowait()
                self._notify_clock_measurement(*raw)
            except Queue.Empty:
                break

        new_time = False
        while 1:
            try:
                self.times.append( self.time_q.get_nowait() )
                new_time = True
            except Queue.Empty:
                break

        if new_time and len(self.times) > 3:
            # require 3 samples to start attempting to fit model
            tdata = np.array(self.times[-100:])
            try:
                self.time_model = get_time_model(tdata[:,1], tdata[:,0], max_residual=0.1)
            except TimeFitError, err:
                self._log.warn('error fitting time_model: %s'%err)
            else:
                self._notify_clockmodel(self.time_model.gain, self.time_model.offset)
                approx_freq = 1.0/self.time_model.gain
                self._log.debug('approximate timer frequency: %s'%(approx_freq,))

    def _notify_framerate(self, expected_trigger_rate):
        self._log.info('framerate: %f' % expected_trigger_rate)

    def _notify_clockmodel(self, gain, offset):
        self._log.info('gain: %s offset: %s' % (gain, offset))

    def _notify_clock_measurement(self, start_timestamp, pulsenumber, fraction_n_of_255, stop_timestamp):
        pass

    def _notify_aout_confirm(self, pulsenumber, fraction_n_of_255, aout0, aout1):
        pass

    def _notify_fatal_error(self, msg):
        self._log.critical(msg)

    def _notify_connected(self, name, device):
        self._log.info("connected to %s" % device)

    def run(self):
        i = 0
        while True:
            if (i % 100) == 0:
                self._get_new_data()
            if i == 2000:
                self._trim_data()
                i = 0
            i += 10
            time.sleep(0.01)

    def set_triggerrate(self, rate_ideal):

        if rate_ideal == 0:
            self._log.info('triggerbox_device: setting FPS to ZERO')
            self.outq.put( ('stop_pulses',) )
            self._clear_data()
            self.expected_trigger_rate = rate_ideal
            self._notify_framerate(self.expected_trigger_rate)
            return

        def get_rate(rate_ideal, prescaler):
            xtal = 16e6 # 16 MHz clock
            base_clock = xtal/float(prescaler)
            new_top_ideal = base_clock/rate_ideal
            new_icr1 = int(np.clip(np.round(new_top_ideal),0,0xFFFF))
            rate_actual = base_clock/new_icr1
            return new_icr1, rate_actual

        self._log.info('received set_triggerrate command with target of %s'%rate_ideal)
        self._clear_data()

        icr1_8, rate_actual_8 = get_rate( rate_ideal, 8)
        icr1_64, rate_actual_64 = get_rate( rate_ideal, 64)

        error_8  = abs(rate_ideal-rate_actual_8)
        error_64 = abs(rate_ideal-rate_actual_64)

        if error_8 < error_64:
            new_icr1 = icr1_8
            rate_actual = rate_actual_8
            prescaler = 8
        else:
            new_icr1 = icr1_64
            rate_actual = rate_actual_64
            prescaler = 64

        self._log.info( 'desired rate %s (actual rate %s) using ICR1_AND_PRESCALER %x %d' % (rate_ideal,
                                                                                            rate_actual,
                                                                                            new_icr1,
                                                                                            prescaler) )
        self.expected_trigger_rate = np.nan
        self._notify_framerate(self.expected_trigger_rate)

        self.outq.put( ('stop_pulses_and_reset',) ) # stop clock, reset pulsecounter
        self.outq.put( ('icr1_and_prescaler', (new_icr1,prescaler)) ) # set triggerrate
        self._clear_data() # clear old data
        self.outq.put( ('start_pulses',) ) # stop clock, reset pulsecounter

        self.expected_trigger_rate = rate_actual
        self._notify_framerate(self.expected_trigger_rate)

    def pause_and_reset(self, pause_duration_seconds):
        orig_value = self.expected_trigger_rate
        self.expected_trigger_rate = np.nan
        self._notify_framerate(self.expected_trigger_rate)

        self.outq.put( ('stop_pulses_and_reset',) ) # stop clock, reset pulsecounter
        self._clear_data() # clear old data
        time.sleep(pause_duration_seconds)
        self._clear_data() # clear old data
        self.outq.put( ('start_pulses',) ) # stop clock, reset pulsecounter

        self.expected_trigger_rate = orig_value
        self._notify_framerate(self.expected_trigger_rate)

    def set_aout_ab_volts(self, aout0_v, aout1_v):
        aout0 = volts_to_dac(aout0_v)
        aout1 = volts_to_dac(aout1_v)
        self.outq.put( ('AOut', aout0, aout1) ) # set AOUT

    def set_aout_ab_raw(self, aout0, aout1):
        self.outq.put( ('AOut', aout0, aout1) ) # set AOUT

if __name__=='__main__':
    import itertools
    logging.basicConfig(level=logging.DEBUG)
    td = TriggerboxDevice('/dev/ttyUSB0')
    for i in itertools.cycle(range(5,200,10)):
        td.set_triggerrate(i)
        time.sleep(10)

