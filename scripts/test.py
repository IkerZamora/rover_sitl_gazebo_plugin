
#!/usr/bin/env python
#import roslib
import rospy

from geometry_msgs.msg import Twist

def publisher():
    pub = rospy.Publisher('/rover/cmd_vel', Twist, queue_size=10)
    rospy.init_node('rover_publisher', anonymous=True)
    #rate = rospy.Rate(10) # 10hz

    twist = Twist()

    while not rospy.is_shutdown():
        input = raw_input("Enter gas, brake and turn (Ex:  1 2 3) :")
        nums = [0.0, 0.0, 0.0]
        #nums = [float(n) for n in input.split()]
        i = 0
        for n in input.split():
            nums[i]=float(n)
            i += 1
        twist.linear.x=nums[0]
        twist.linear.z=nums[1]
        twist.angular.x=nums[2]

        pub.publish(twist)
        #rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass