#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
import time

from random import randint, random, uniform



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    return


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data


    # given a point, returns the row-major order index 
    # of the cell that the point lies in on the map
    # returns origin if point not in map
    def point_to_map(self, point):
        map_origin = self.map.info.origin
        horizontal_dist = (point.x - map_origin.position.x)/self.map.info.resolution
        vertical_dist = (point.y - map_origin.position.y)/self.map.info.resolution
        width_val = int(math.floor(horizontal_dist))
        height_val = int(math.floor(vertical_dist))

        # check point is in bounds
        if width_val < 0 or width_val >= self.map.info.width:
            return 0
        elif height_val < 0 or height_val >= self.map.info.height:
            return 0
        
        row_major_order_index =  width_val + height_val*self.map.info.width
        return row_major_order_index


    # checks if a particle is within the house and not on an object
    def valid_particle(self, point):
        if self.map.data[self.point_to_map(point)] == 0:
            return True

    # generates a random particle within the house
    def gen_random_particle(self):
        # randomly generate a point until we get one in bounds
        pt = Point(uniform(-10, 10), uniform(-10, 10), 0)
        while not self.valid_particle(pt):
            pt = Point(uniform(-10, 10), uniform(-10, 10), 0)
        # randomly generate orientation
        quat_array = quaternion_from_euler(0, 0, uniform(0, 360))
        orientation = Quaternion(quat_array[0], quat_array[1], quat_array[2], quat_array[3])
        pose = Pose(pt, orientation)
        
        return Particle(pose, w =1)


    def initialize_particle_cloud(self):
        #sleep a little to wait for map to publish
        time.sleep(2)
        #Initialize self.num_particles particles
        for _ in range(self.num_particles):
            self.particle_cloud.append(self.gen_random_particle())

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        total_weight = 0
        # get sum of weights
        for particle in self.particle_cloud:
            total_weight += particle.w
        # divide by sum
        for particle in self.particle_cloud:
            particle.w = particle.w/total_weight



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        pass
        # TODO



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        pass
        # TODO


    
    def update_particle_weights_with_measurement_model(self, data):
        pass
        # TODO


        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        xpos_delta = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        ypos_delta = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        angle_delta = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        for particle in self.particle_cloud:
            particle.pose.position.x += xpos_delta
            particle.pose.position.y += ypos_delta
            new_euler_angle = get_yaw_from_pose(particle.pose) + angle_delta
            new_qtrn_arr = quaternion_from_euler(0, 0, new_euler_angle)
            particle.pose.orientation = Quaternion(new_qtrn_arr[0], new_qtrn_arr[1], new_qtrn_arr[2], new_qtrn_arr[3])




if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









