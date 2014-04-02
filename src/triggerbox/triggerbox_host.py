#!/usr/bin/env python
import argparse
import numpy as np

import roslib; roslib.load_manifest('triggerbox')

import rospy
from triggerbox.msg import TriggerClockModel, AOutVolts, AOutRaw
from triggerbox.triggerbox_device import TriggerboxDevice

import std_msgs

class TriggerboxHost(TriggerboxDevice):
    '''an in-process version of the triggerbox client with identical API'''
    def __init__(self, device, write_channel_name, channel_name):

        self._gain = np.nan
        self._offset = np.nan
        self._expected_framerate = None

        self.pub_time = rospy.Publisher('~time_model', TriggerClockModel)
        self.pub_rate = rospy.Publisher('~expected_framerate', std_msgs.msg.Float32)

        super(TriggerboxHost,self).__init__(device, write_channel_name, channel_name)

        rospy.Subscriber('~set_triggerrate', std_msgs.msg.Float32,
                lambda _msg: self.set_triggerrate(msg.data))
        rospy.Subscriber('~pause_and_reset', std_msgs.msg.Float32,
                lambda _msg: self.pause_and_reset(msg.data))
        rospy.Subscriber('~aout_volts', AOutVolts,
                lambda _msg: self.set_aout_ab_volts(self,_msg.aout0,_msg.aout1))
        rospy.Subscriber('~aout_raw', AOutRaw,
                lambda _msg: self.set_aout_ab_raw(self,_msg.aout0,_msg.aout1))

        # emit expected frame rate every 5 seconds
        rospy.Timer(rospy.Duration(5.0), self._on_emit_framerate)

    def _on_emit_framerate(self, _=None):
        if self._expected_framerate is not None:
            self.pub_rate.publish(self._expected_framerate)

    def notify_framerate(self, expected_trigger_rate):
        self._expected_framerate = expected_trigger_rate
        self.pub_rate.publish(self._expected_framerate)

    def notify_clockmodel(self, gain, offset):
        self._gain = gain
        self._offset = offset
        self.pub_time.publish(self._gain, self._offset)

    def notify_fatal_error(self, msg):
        rospy.logfatal(msg)
        rospy.signal_shutdown(msg)

    #ClientAPI
    def have_estimate(self):
        return (not np.isnan(self._gain)) and (not np.isnan(self._offset))

    def wait_for_estimate(self):
        while not self.have_estimate():
            rospy.loginfo('triggerbox_host: waiting for clockmodel estimate')
            time.sleep(0.5)

    def timestamp2framestamp(self, timestamp ):
        return (timestamp-self._offset)/self._gain

    def framestamp2timestamp(self, framestamp ):
        return framestamp*self._gain + self._offset

    def get_frames_per_second(self,wait_for_valid=True):
        while True:
            result = self._expected_framerate
            if result is not None:
                break
            if not wait_for_valid:
                break
            time.sleep(0.01)
        return result

    def set_frames_per_second(self,value):
        rospy.loginfo('triggerbox_host: setting FPS to %s' % value)
        self.set_triggerrate(value)

    def synchronize(self, pause_duration_seconds=2 ):
        rospy.loginfo('triggerbox_host: synchronizing')
        self.pause_and_reset(pause_duration_seconds)

        msg = std_msgs.msg.Float32( pause_duration_seconds )
        self.sync_pub.publish( msg )

if __name__=='__main__':
    rospy.init_node('triggerbox_host')
    tb = TriggerboxHost('/dev/ttyUSB0', None, None)
    tb.wait_for_estimate()
    rospy.spin()

