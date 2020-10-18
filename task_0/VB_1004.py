#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Turtle:
        # Initializing the Class
        def __init__(self):    
    
            # Intializing the turtle_revolve node
            rospy.init_node('node_turtle_revolve', anonymous=True)
            
	    # Subscribe to /turtle1/cmd_vel topic
            self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
            
	    # Create a publisher handle for the /turtle1/pose topic
            self.pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
            
	    # Creating the Pose message 
            self.Pos = Pose()
            self.rate = rospy.Rate(1000)            
    
        def moveInCircle(self):   
         
            # Creating Twist message by initializing the values 
            Vel = Twist()                        
                     
            Vel.linear.x = 1.0
            Vel.linear.y = 0.0
            Vel.linear.z = 0.0
                    
            Vel.angular.x = 0.0
            Vel.angular.y = 0.0
            Vel.angular.z = 1.0
            
	    # Publish velocity messages with a delay of rate.sleep()
            while not rospy.is_shutdown():
                    self.velocity_publisher.publish(Vel)
		    # Check if half circle is completed
                    if(self.Pos.theta < 0):
			    # Check if initial position is reached
                            if((2 * 3.1415) + self.Pos.theta >= 6.27):                                
                                Vel.linear.x = 0.0                                
                                Vel.angular.z = 0.0
                                self.velocity_publisher.publish(Vel)
                                rospy.loginfo('Goal Acheived')
                                break

                    self.rate.sleep()
            
        def pose_callback(self, msg):
            # Assigning message to Pos  
            self.Pos = msg
	    if msg.theta >= 0:
            	rospy.loginfo("Moving in a circle:\n%f", msg.theta)
	    else:
		rospy.loginfo("Moving in a circle:\n%f", (2 * 3.1415) + msg.theta)

if __name__ == '__main__':
    try:
        Turt = Turtle()
        Turt.moveInCircle()
    except rospy.ROSInterruptException:
        pass
