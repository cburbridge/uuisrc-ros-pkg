#!/usr/bin/env python
import roslib; roslib.load_manifest('pointcloud_compress')
import rospy
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud2
import cStringIO
import zlib

class Decompressor:
	def __init__(self, node):
		self.node = node
		
		# Get the parameters
		self.input_cloud = rospy.get_param("~input",'/camera/depth/points2_compressed')
		self.output_cloud= rospy.get_param("~output", '/camera/depth/points2_decompressed')

		# Friendly info
		rospy.loginfo("Point Cloud Decompressor started.")
		rospy.loginfo("Point Cloud Decompressor listening:   %s.",self.input_cloud)
		rospy.loginfo("Point Cloud Deompressor publishing:  %s.",self.output_cloud)
		
		self.decompressed_msg = PointCloud2()
		rospy.Subscriber(self.input_cloud, ByteMultiArray, self.receive_cloud)
		self.publisher = rospy.Publisher(self.output_cloud,PointCloud2)

	# When a point cloud is received, decompress and deserialise it and publish it again.
	def receive_cloud(self, data):
		self.decompressed_msg.deserialize(zlib.decompress(data.data))
		self.publisher.publish(self.decompressed_msg)

if __name__ == '__main__':
	node = rospy.init_node('cloud_decompressor',anonymous=True)
	c = Decompressor(node)
	
	rospy.spin()


