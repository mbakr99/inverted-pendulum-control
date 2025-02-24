# /usr/bin/python3

import rospy
import sys

if __name__ == '__main__':
    rospy.init_node('test_node')
    
    if len(sys.argv) > 1:
        arguments = ""
        for arg in sys.argv:
            arguments += arg + " "
        rospy.loginfo(f"args are {arguments}")
        
    else:
        rospy.loginfo("no args passed")
    
    rospy.signal_shutdown("finished")
    
    