#!/usr/bin/env python
import numpy as np
import time

import roslib; roslib.load_manifest('triggerbox')

import rospy
from triggerbox.msg import AOutRaw

def main():
    rospy.init_node('triggerbox_host')
    aout_raw_pub = rospy.Publisher('/trig1/aout_raw', AOutRaw)

    cnt = 0
    while 1:

        msg = AOutRaw()
        msg.aout0 = cnt
        msg.aout1 = 4095-cnt
        aout_raw_pub.publish( msg )

        cnt += 200
        if cnt > 4095:
            cnt = 0

        time.sleep(0.1)

if __name__=='__main__':
    main()
