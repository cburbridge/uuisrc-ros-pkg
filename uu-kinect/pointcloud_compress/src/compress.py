#!/usr/bin/env python
import roslib; roslib.load_manifest('pointcloud_compress')
import rospy
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud2
import cStringIO
import zlib
import thread

class Compressor:
	def __init__(self, node):
		self.node = node

		# Get the parameters
		self.input_cloud = rospy.get_param("~input",'/camera/depth/points2')
		self.output_cloud= rospy.get_param("~output", '/camera/depth/points2_compressed')
		self.compress_hz = rospy.get_param("~hz",1)
		self.compress_level= rospy.get_param("~level",6)

		# Friendly info
		rospy.loginfo("Point Cloud Compressor started.")
		rospy.loginfo("Point Cloud Compressor listening:   %s.",self.input_cloud)
		rospy.loginfo("Point Cloud Compressor publishing:  %s.",self.output_cloud)
		rospy.loginfo("Point Cloud Compressor compression: %d.",self.compress_level)
		rospy.loginfo("Point Cloud Compressor frequency:   %d.",self.compress_hz)

		self.compressed_msg = ByteMultiArray()
		self.compressed_msg.layout.dim.append(MultiArrayDimension())
		rospy.Subscriber(self.input_cloud, PointCloud2, self.receive_cloud)
		self.publisher = rospy.Publisher(self.output_cloud,ByteMultiArray)
		self.buf=None #cStringIO.StringIO()
		self.lock=thread.allocate_lock()

	# When a point cloud is received, serialise it and store it
	def receive_cloud(self, data):
		self.lock.acquire() 		# lock mutex - maybe we have a thread
		self.buf = cStringIO.StringIO()  # store it in buffer
		data.serialize(self.buf)
		self.lock.release()


if __name__ == '__main__':
	node = rospy.init_node('cloud_compressor',anonymous=True)
	c = Compressor(node)
	rate = rospy.Rate(c.compress_hz)
	while not rospy.is_shutdown():
		rate.sleep()
		if c.buf == None:
			continue

		c.lock.acquire()
		stuffed = zlib.compress(c.buf.getvalue(),c.compress_level)
#		print len(c.buf.getvalue()), " ==> ", len(stuffed)
		c.compressed_msg.data=stuffed
		c.compressed_msg.layout.dim[0].size=len(stuffed)
		c.publisher.publish(c.compressed_msg)
	
		c.lock.release()

	rospy.spin()


