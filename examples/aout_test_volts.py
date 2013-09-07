#!/usr/bin/env python
import numpy as np
import time

import roslib; roslib.load_manifest('triggerbox')

import rospy
from triggerbox.msg import AOutVolts

def main():
    rospy.init_node('triggerbox_host')
    aout_volts_pub = rospy.Publisher('/trig1/aout_volts', AOutVolts)

    value = 0.0
    while 1:

        msg = AOutVolts()
        msg.aout0 = value
        msg.aout1 = 4.0-value
        aout_volts_pub.publish( msg )

        value += 0.1
        if value > 4.0:
            value = 0

        time.sleep(0.1)

if __name__=='__main__':
    main()
