from pymavlink import mavutil
import rospy
from std_msgs.msg import Float64
number=' '
def cb_context(msg):
	number=msg.data
def cb(msg):
	if msg.data==777.777:
		connect=mavutil.mavlink_connection('/dev/ttyTHS1',baud=115200)
	#connect.target_system=255
	#connect=mavutil.mavlink_connection('udpout:localhost:14445',source_system=1)
		connect.wait_heartbeat()
		print('connect successfully')
		for i in range(20):
			connect.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,"target number123".encode())

rospy.init_node("send")
rospy.Subscriber('send_topic', Float64, cb, queue_size=10)
rospy.Subscriber('final_result', String, cb_context, queue_size=10)
rospy.spin()

