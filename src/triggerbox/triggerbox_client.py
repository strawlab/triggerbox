import time
import numpy as np

import roslib
roslib.load_manifest('triggerbox')
import rospy

from triggerbox.api import TriggerboxAPI
from triggerbox.time_model import get_time_model, TimeFitError
from triggerbox.msg import TriggerClockModel, TriggerClockMeasurement
from triggerbox.srv import SetFramerate

import std_msgs.msg

class TriggerboxClient(TriggerboxAPI):
    '''a client to coordinate synchronization and time reconstruction'''
    def __init__(self, host_node='/triggerbox_host' ):
        rospy.Subscriber(host_node+'/time_model',
                         TriggerClockModel,
                         self._on_trigger_clock_model)
        rospy.Subscriber(host_node+'/raw_measurements',
                         TriggerClockMeasurement,
                         self._on_trigger_clock_measurement)
        rospy.Subscriber(host_node+'/expected_framerate',
                         std_msgs.msg.Float32,
                         self._on_expected_framerate)
        self.fps_pub = rospy.Publisher(
                         host_node+'/set_triggerrate',
                         std_msgs.msg.Float32,
                         latch=True)
        self.sync_pub = rospy.Publisher(
                         host_node+'/pause_and_reset',
                         std_msgs.msg.Float32)

        self._fps_srv_url = host_node+'/set_framerate'
        self.fps_srv = rospy.ServiceProxy(
                         self._fps_srv_url,
                         SetFramerate)

        self._host_node = host_node

        self._gain = np.nan
        self._offset = np.nan
        self._expected_framerate = None

        self._connected = False

    def _on_expected_framerate(self,msg):
        value = msg.data
        if np.isnan(value):
            self._expected_framerate = None
        else:
            self._expected_framerate = value
        self._api_callback(self.framerate_callback, self._expected_framerate)

    def _on_trigger_clock_model(self,msg):
        self._gain = msg.gain
        self._offset = msg.offset
        self._api_callback(self.clockmodel_callback, self._gain, self._offset)

        if not self._connected:
            have_estimate = self.have_estimate()
            if have_estimate:
                self._api_callback(self.connected_callback, self._host_node, "ROS")
                self._connected = True

    def _on_trigger_clock_measurement(self,msg):
        self._api_callback(self.clock_measurement_callback,
                msg.start_timestamp, msg.pulsenumber, msg.fraction_n_of_255, msg.stop_timestamp)

    def have_estimate(self):
        return (not np.isnan(self._gain)) and (not np.isnan(self._offset))

    def wait_for_estimate(self):
        while not self.have_estimate():
            rospy.loginfo('triggerbox_client: waiting for clockmodel estimate')
            rospy.sleep(0.5)

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
            rospy.sleep(0.1)
        return result

    def set_frames_per_second(self,value):
        rospy.loginfo('trigger_client: setting FPS to %s' % value)
        self._expected_framerate = None # clear old value
        msg = std_msgs.msg.Float32(value)
        self.fps_pub.publish(msg)

    def set_frames_per_second_blocking(self,value):
        rospy.wait_for_service(self._fps_srv_url)
        rospy.loginfo('trigger_client: setting FPS to %s' % value)
        self._expected_framerate = None # clear old value
        self.fps_srv(value)

    def synchronize(self, pause_duration_seconds=2 ):
        rospy.loginfo('trigger_client: requesting synchronization')
        msg = std_msgs.msg.Float32( pause_duration_seconds )
        self.sync_pub.publish( msg )

if __name__=='__main__':
    rospy.init_node('triggerbox_client')
    tb = TriggerboxClient()
    tb.connected_callback = lambda _n, _d: rospy.loginfo("connected to '%s' via %s" % (_n, _d))
    #tb.clock_measurement_callback = lambda *args: rospy.loginfo("raw clock measurement: %r" % (args,))
    tb.set_frames_per_second_blocking(25.0)
    tb.wait_for_estimate()
    rospy.spin()

