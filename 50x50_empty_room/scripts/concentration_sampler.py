#!/usr/bin/python
import math
import numpy as np
import rospy
import tf2_ros
import scipy.io
from geometry_msgs.msg import TransformStamped
from olfaction_msgs.msg import gas_sensor
from matplotlib import path

class tf_publisher:
    def __init__(self, sensor_topic, sensor_tf, map_tf, waypoints):
        self.waypoints = waypoints
        self.sample_time = 2.5
        self.parent_tf = map_tf
        self.sensor_tf = sensor_tf
        self.sensor_topic = sensor_topic

        self.GT_map = np.zeros((len(waypoints),))

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.frame_id = self.parent_tf
        self.tf_msg.child_frame_id = self.sensor_tf
        self.tf_msg.transform.translation.x = 0
        self.tf_msg.transform.translation.y = 0
        self.tf_msg.transform.translation.z = 0
        self.tf_msg.transform.rotation.x = 0
        self.tf_msg.transform.rotation.y = 0
        self.tf_msg.transform.rotation.z = 0
        self.tf_msg.transform.rotation.w = 1
        msg = []
        while not msg:
            try:
                msg = rospy.wait_for_message(self.sensor_topic, gas_sensor,0.1)
            except:
                self.tf_broadcaster.sendTransform(self.tf_msg)

        self.subscriber = rospy.Subscriber(sensor_topic, gas_sensor, self.callback)

    def callback(self, msg):
        self.tf_broadcaster.sendTransform(self.tf_msg)

    def publish_tf(self):
        rospy.loginfo('Publishing waypoint x,y,z: {}'.format(waypoints))
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.frame_id = self.parent_tf
        self.tf_msg.child_frame_id = self.sensor_tf
        self.tf_msg.transform.translation.x = waypoints[0]
        self.tf_msg.transform.translation.y = waypoints[1]
        self.tf_msg.transform.translation.z = waypoints[2]
        self.tf_msg.transform.rotation.x = 0
        self.tf_msg.transform.rotation.y = 0
        self.tf_msg.transform.rotation.z = 0
        self.tf_msg.transform.rotation.w = 1

        msg = []

        msg = rospy.wait_for_message(self.sensor_topic, gas_sensor)

        t_start = rospy.get_time()
        sensor_buffer = []
        while rospy.get_time() - t_start < self.sample_time:
            msg = rospy.wait_for_message(self.sensor_topic, gas_sensor)
            sensor_buffer.append(msg.raw)           
            self.GT_map = np.mean(np.array(sensor_buffer))


if __name__ == "__main__":
    rospy.init_node('sweep_tf', anonymous=True)

    sensor_topic = rospy.get_param('~sensor_topic', '/PID/Sensor_reading')
    rospy.loginfo('Sensor topic: {}'.format(sensor_topic))

    sensor_tf = rospy.get_param('~sensor_tf', '/pid_frame')
    rospy.loginfo('Sensor tf: {}'.format(sensor_tf))

    map_tf = rospy.get_param('~map_tf', '/map')
    rospy.loginfo('Using parent tf: {}'.format(map_tf))

    waypoints = [5,5,3]
    
    setpoint_obj = tf_publisher(sensor_topic, sensor_tf, map_tf, waypoints)
    setpoint_obj.waypoints


    start = rospy.get_time()
    setpoint_obj.publish_tf()

    rospy.loginfo('Time taken to run sweep: {}s'.format(rospy.get_time()-start))
    np.save('sample_waypoints.npy', setpoint_obj.waypoints)
    np.save('sample_data.npy', setpoint_obj.GT_map)
    scipy.io.savemat('GT_data.mat', {'GT_conc':setpoint_obj.GT_map, 'GT_waypoints':setpoint_obj.waypoints})

    rospy.spin()

