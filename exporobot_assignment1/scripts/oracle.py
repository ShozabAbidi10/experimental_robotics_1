#! /usr/bin/env python2

## @package exporobot_assignment1
# \file motion_controller.py
# \brief This file contains code for controlling motion of the robot.
# \author Shozab Abidi
# \version 1.0
# \date 13/11/2021
#
# \details
#
# Subscribes : <BR>
# 	° /odom
# 	° /control_speed
#
# Publishers : <BR>
# 	° /odom
# 	° /control_speed
# 	° /cancel_goals
# ° /success_goals
#
# Service : <BR>
# ° /go_to_point

def main():

    rospy.init_node('oracle')
    service = rospy.Service('/go_to_point', Position, go_to_point)
    rospy.spin()

if __name__ == '__main__':
    main()
