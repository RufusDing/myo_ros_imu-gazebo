#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu

if __name__ == '__main__':

    rospy.init_node('gazebo_driver', anonymous=True)
    
    # Publish to the turtlesim movement topic
    tsPub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

    def strength(imuarg,mode=0):
	#emgArr=emgArr1.data
	# Define proportional control constant:
	#K = 0.005
	# Get the average muscle activation of the left, right, and all sides

	#aveRight=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
	#aveLeft=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
	#ave=(aveLeft+aveRight)/2
		forwardorbackwrd = imuarg.orientation.z
		leftorright = imuarg.orientation.y
		#x = imuarg.orientation.x
		# If all muscles activated, drive forward exponentially

		if mode == 0:
			if forwardorbackwrd > 0.3:
				tsPub.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,0)))
			elif forwardorbackwrd < -0.3:
				tsPub.publish(Twist(Vector3(-0.1,0,0),Vector3(0,0,0)))
			# If only left muscles activated, rotate proportionally
			elif leftorright > 0.3:
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.2)))
			# If only right muscles activated, rotate proportionally
			elif leftorright < -0.3:
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-0.2)))
			#circle    
			#elif x < 0.7:
			    #tsPub.publish(Twist(Vector3(1.0,1.0,0),Vector3(0,0,1.2)))	
			# If not very activated, don't move (high-pass filter)    
			else: 
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
		elif mode == 1:
			if forwardorbackwrd < -0.3 :
			    tsPub.publish(Twist(Vector3(0.1*(-forwardorbackwrd-0.3),0,0),Vector3(0,0,0)))
			elif forwardorbackwrd > 0.3:
				tsPub.publish(Twist(Vector3(-0.1*(forwardorbackwrd-0.3),0,0),Vector3(0,0,0)))
			# If only left muscles activated, rotate proportionally
			elif leftorright > 0.3:
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.2*(leftorright-0.3))))
			# If only right muscles activated, rotate proportionally
			elif leftorright < -0.3:
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-0.2*(-leftorright-0.3))))	
			# If not very activated, don't move (high-pass filter)    
			else: 
			    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

    rospy.Subscriber("myo_imu", Imu, strength)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
