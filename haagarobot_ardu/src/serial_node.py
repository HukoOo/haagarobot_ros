#!/usr/bin/env python

import rospy
from rosserial_python import SerialClient, RosSerialServer
import multiprocessing


import sys


if __name__=="__main__":


	rospy.init_node("arduino_node")
	rospy.loginfo("ROS Serial Python Node")


	port_name = rospy.get_param('~port','/dev/ttyACM0')
	baud = int(rospy.get_param('~baud','115200'))


	sys.argv = rospy.myargv(argv=sys.argv)
	if len(sys.argv) > 1 :
		port_name = sys.argv[1]
		baud = int(sys.argv[2])


	rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
	client = SerialClient(port_name, baud)
	try:
		client.run()
	except KeyboardInterrupt:
		pass
