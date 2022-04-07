import rospy, rostopic
rospy.init_node('subscriber_multi')
r = rostopic.ROSTopicHz(-1)
s = rospy.Subscriber('/S0/mavros/local_position/pose', rospy.AnyMsg, r.callback_hz, callback_args='/S0/mavros/local_position/pose')
rospy.sleep(1)
r.print_hz(['/S0/mavros/local_position/pose'])