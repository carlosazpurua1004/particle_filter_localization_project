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
import copy

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


def draw_random_sample(choice_list, probabilities, num_samples: int):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # we divide the interval (0, 1) into intervals accoring to the probabilities, and for a random point
    # we perform a binary search on this interval to find the interval it lies in
    partial_sums = []
    psum = 0
    #compute partial sums of the probabiltiies so we can binary search the interval
    for prob in probabilities:
        psum += prob
        partial_sums.append(psum)
    
    #draw samples
    samples = []
    for _ in range(num_samples):
        random_point = random_sample()
        lower_bound = 0
        upper_bound = len(choice_list) - 1
        # binary search to find the interval the random point lies in
        while lower_bound != upper_bound:
            mid_point = (lower_bound + upper_bound)//2
            if random_point < partial_sums[mid_point] :
                upper_bound = mid_point
            elif random_point >= partial_sums[mid_point+1]:
                lower_bound = mid_point + 1
            else:
                lower_bound = mid_point + 1
                upper_bound = mid_point + 1
        #append sample to list of samples
        samples.append(copy.deepcopy(choice_list[lower_bound]))

    return samples


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
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None
        
        # indicate the angles that we plan on checking for particle weight update
        self.directions_to_check = [(math.pi/2*index) for index in range(4)]

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


    # given a row, column index, get the value of the map at those
    # coordinates, and if out of bounds return -1
    def get_map_val(self, row_index:int, col_index:int) -> int:
        # check point is in bounds
        if row_index < 0 or row_index >= self.map.info.width:
            return -1
        elif col_index < 0 or col_index >= self.map.info.height:
            return -1
        
        return self.map.data[row_index + (col_index * self.map.info.width)]

    # given a point, returns the indices of the cell
    # that the point lies in on the map as (row, column)
    def point_to_map_indices(self, point:Point) -> (int, int):
        map_origin = self.map.info.origin
        horizontal_dist = (point.x - map_origin.position.x)/self.map.info.resolution
        vertical_dist = (point.y - map_origin.position.y)/self.map.info.resolution
        row_index = int(math.floor(horizontal_dist))
        col_index = int(math.floor(vertical_dist))

        return (row_index, col_index)


    # checks if a particle is within the house and not on an object
    def valid_particle(self, point:Point) -> bool:
        if self.get_map_val(*(self.point_to_map_indices(point))) == 0:
            return True

    # generates a random particle within the house
    def gen_random_particle(self) -> Particle:
        # randomly generate a point until we get one in bounds
        pt = Point(uniform(-10, 10), uniform(-10, 10), 0)
        while not self.valid_particle(pt):
            pt = Point(uniform(-10, 10), uniform(-10, 10), 0)
        # randomly generate orientation
        quat_array = quaternion_from_euler(0, 0, uniform(0, 2*math.pi))
        orientation = Quaternion(quat_array[0], quat_array[1], quat_array[2], quat_array[3])
        pose = Pose(pt, orientation)
        
        return Particle(pose, w=1)

    # call in initialize particle cloud to test update weights
    # clears all but a few particles, and puts one particle on the robots
    # starting position
    def test_update_weights(self):
        # list of test particles, including a particle in the same pose as initial posiiton of robot 
        test_particles_vals = [(-3, 1, 0), (0, 0, 0), (-1, 3, math.pi/2), (1, 2, math.pi)]*100
        self.particle_cloud = []
        for p_vals in test_particles_vals:
            quat_array = quaternion_from_euler(0, 0, p_vals[2])
            orientation = Quaternion(quat_array[0], quat_array[1], quat_array[2], quat_array[3])
            robot_pose = Pose(Point(p_vals[0],p_vals[1],0), orientation)
            self.particle_cloud.append(Particle(robot_pose, 1))
        self.num_particles = len(test_particles_vals)

    
    def initialize_particle_cloud(self):
        #sleep a little to wait for map to publish
        time.sleep(2)
        #Initialize self.num_particles particles
        for _ in range(self.num_particles):
            self.particle_cloud.append(self.gen_random_particle())
        #self.test_update_weights()

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
            #print(particle.pose.position, "\nw:", particle.w)



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
        self.particle_cloud = draw_random_sample(self.particle_cloud, [p.w for p in self.particle_cloud], self.num_particles)


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
        # initialize variables for summing average particle positions/orientations
        x = y = z = ox = oy = oz = ow = 0 
        n = len(self.particle_cloud)
        # loop through all particles to sum up position and orientation values
        for particle in self.particle_cloud:
            p = particle.pose

            x += p.position.x
            y += p.position.y
            z += p.position.z

            ox += p.orientation.x
            oy += p.orientation.y
            oz += p.orientation.z
            ow += p.orientation.w

            

        # calculate average positions/orientations and update estimated position
        self.robot_estimate.position.x = x/n
        self.robot_estimate.position.y = y/n
        self.robot_estimate.position.z = z/n

        self.robot_estimate.orientation.x = ox/n
        self.robot_estimate.orientation.y = oy/n
        self.robot_estimate.orientation.z = oz/n
        self.robot_estimate.orientation.w = ow/n

        return


    # given a particle and an angle, estimate the distance to the 
    # nearest object in the direction of that angle relative to the pos,
    # i.e. estimate the lidar reading from that particle
    def estimate_particle_lidar(self, particle:Particle, angle:int) -> float:
        map_indices = self.point_to_map_indices(particle.pose.position)
        start_row_index = map_indices[0]
        start_col_index = map_indices [1]
        # this gives us the angle to check relative to the positive x-axis in radians
        adjusted_angle = (angle) + get_yaw_from_pose(particle.pose)
        map_val = self.get_map_val(start_row_index, start_col_index)
        # iterate across map
        step_size = 0.5
        distance = 0
        ray_row_index = map_indices[0]
        ray_col_index = map_indices [1]
        while map_val == 0:
            ray_row_index = start_row_index + int(round(distance*np.cos(adjusted_angle)))
            ray_col_index = start_col_index + int(round(distance*np.sin(adjusted_angle)))
            map_val = self.get_map_val(ray_row_index, ray_col_index)
            distance += step_size
        # not necessary, but add on an amount proportional to how full the square is to better estimate
        distance += map_val/100
        # maybe write some code here to avoid immediately stopping rays that are parallel to walls
        return (distance)*self.map.info.resolution

    
    def update_particle_weights_with_measurement_model(self, data):
        for particle in self.particle_cloud:
            difference_sum = 0
            # for every angle to check, calculate difference between lidar values and estimated value
            for angle in self.directions_to_check:
                difference_sum += abs(min(data.ranges[int(angle*math.pi/180)], data.range_max) - min(self.estimate_particle_lidar(particle, angle), data.range_max))
            particle.w = 1/difference_sum
        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        xpos_delta = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        ypos_delta = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        magnitude = math.sqrt(xpos_delta**2 + ypos_delta **2)
        angle_delta = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        for particle in self.particle_cloud:
            # generate some noise for particle movement
            movement_noise = np.random.normal(0, 0.05, 2)
            # noise for particle direction, approx 3degree standard deviation 
            angle_noise = np.random.normal(0, 0.051, 1) 
            particle_dir = get_yaw_from_pose(particle.pose)
            # We estimate that the difference between the motion vector of the particle and the orientation
            # is angle_delta/2, and use this to transform the given motion vector to the coordinates of the particle
            particle.pose.position.x += magnitude*math.cos(particle_dir - angle_delta/2) + movement_noise[0]
            particle.pose.position.y += magnitude*math.sin(particle_dir - angle_delta/2) + movement_noise[1]
            # update angle of particle based on turn
            new_euler_angle = particle_dir + angle_delta + angle_noise[0]
            new_qtrn_arr = quaternion_from_euler(0, 0, new_euler_angle)
            particle.pose.orientation = Quaternion(new_qtrn_arr[0], new_qtrn_arr[1], new_qtrn_arr[2], new_qtrn_arr[3])




if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









