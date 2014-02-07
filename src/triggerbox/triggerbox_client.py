import time
import numpy as np

import roslib
roslib.load_manifest('triggerbox')
import rospy

from triggerbox.time_model import get_time_model, TimeFitError
from triggerbox.msg import TriggerClockModel

import std_msgs.msg

class TriggerboxClient:
    '''a client to coordinate synchronization and time reconstruction'''
    def __init__(self, host_node='/triggerbox_host' ):
        rospy.Subscriber(
            host_node+'/time_model',
            TriggerClockModel,
            self._on_trigger_clock_model)
        rospy.Subscriber(
            host_node+'/expected_framerate',
            std_msgs.msg.Float32,
            self._on_expected_framerate)
        self.fps_pub = rospy.Publisher( host_node+'/set_triggerrate',
                                        std_msgs.msg.Float32,
                                        latch=True )
        self.sync_pub = rospy.Publisher( host_node+'/pause_and_reset',
                                         std_msgs.msg.Float32 )

        self.gain = np.nan
        self.offset = np.nan
        self.expected_framerate = None

    def _on_expected_framerate(self,msg):
        value = msg.data
        if np.isnan(value):
            self.expected_framerate = None
        else:
            self.expected_framerate = value

    def _on_trigger_clock_model(self,msg):
        self.gain = msg.gain
        self.offset = msg.offset

    def wait_for_estimate(self):
        while not self.have_estimate():
            rospy.loginfo('waiting for triggerbox estimate')
            time.sleep(0.5)

    def have_estimate(self):
        return (not np.isnan(self.gain)) and (not np.isnan(self.offset))

    def timestamp2framestamp(self, timestamp ):
        return (timestamp-self.offset)/self.gain

    def framestamp2timestamp(self, framestamp ):
        if np.nan in (self.gain,self.offset):
            print "fs,g,o: %r %r %r" % (framestamp,self.gain,self.offset)
        return framestamp*self.gain + self.offset

    def get_frames_per_second(self,wait_for_valid=True):
        while True:
            result = self.expected_framerate
            if result is not None:
                break
            if not wait_for_valid:
                break
            time.sleep(0.01)
        return result

    def set_frames_per_second(self,value):
        rospy.loginfo('trigger_client: setting FPS to %s' % value)
        self.expected_framerate = None # clear old value
        msg = std_msgs.msg.Float32(value)
        self.fps_pub.publish(msg)

    def synchronize(self, pause_duration_seconds=2 ):
        rospy.loginfo('trigger_client: requesting synchronization')
        msg = std_msgs.msg.Float32( pause_duration_seconds )
        self.sync_pub.publish( msg )
