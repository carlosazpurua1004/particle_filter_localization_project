# particle_filter_localization_project
![gif](particle_filter.gif)

### Objectives Description
The objective of this project is to apply our understanding of the particle filter algorithm to solve the problem of robot localization.  It is critical for a robot to understand its position relative to its environment in order to function properly.  The particle filter algorithm is a way to accurately estimate its location in a mapped environment.

### High-Level Description
We solved the problem of robot localization by implementing the particle filter algorithm shown in our lectures and coursework. In our code, there are five high level components to our approach:
1. `Initialization of Particle Cloud`: Initialize a cloud of 2000 random particles with random poses in the mapped environment
2. `Movement Model`: Upon recieving a movement large enough from the odom topic, update the pose of all the particles according to how the robot moved. Perform steps 3-6
3. `Measurement Model`: Compute a weight for each particle based on how closely the robot lidar readings match what they would be if the robot
4. `Resampling`: Normalize particles by weight to get a probability for each particle, and resample all the particles according to these probabilties
5. `Updating Estimated Robot Pose`: Updating estimated robot pose according to the average pose of the all the particles


### Step Descriptions
#### Initialization of Particle Cloud
##### Code Location
This functionality is implemented with the helpers below, upon initialization of the particle filter object. Note that we refer
to some of these helpers in future descriptions.
* `get_map_val`: Lime 157
* `point_to_map_indices`: Line 156
* `valid_particle()`: Line 179
* `gen_random_particle()`: Line 184
* `initialize_particle_cloud()`: Line 211
##### Functions/Code Description
* `get_map_val`: This helper takes row and column map indices, checks if they are in bounds and returns the appropriate map value.
* `point_to_map_indices`: This helper converts a point in the coordinates of our ROS world to indices for the map.
* `valid_particle()`: This helper indicates whether a particle is within the house by checking that it is in a square with map value of 0.
* `gen_random_particle()`: This helper function generates a particle with a random location and orientation within the house, using the above helpers.
* `initialize_particle_cloud()`: This function uses gen_random_particle() to generate the number of particles we desire, all within the house.

#### Movement Model
##### Code Location
This functionality is implemented entirely within the function below, which is called upon recieving a notice from the odom listener about a change in the robots pose above some threshhold values.
* `update_particles_with_motion_model()`: Line 402
##### Functions/Code 
* `update_particles_with_motion_model()`: We compute the difference of the previously recieved robot pose to the one we just recieved to get a magnitude for the direction travelled by a robot, and then estimate that the direction of the movement vector is halfway between its old orientation and new orientation (we could compute it directly, but this keeps things simple). Then for every particle, we update its pose by moving it along the same movement vector relative to the particles respective pose, and changing its angle by the same change we saw in the odometry. We also incorporate noise here which we describe later.

#### Measurement Model
##### Code Location
This functionality is mostly implemented in the two functions below, with some use of the helpers from the initialization step. 
* `estimate_particle_lidar()`: Line 370
* `update_particles_with_measurement_model()`: Line 393
##### Functions/Code Description
* `estimate_particle_lidar()`: This helper function is used to estimate distance between a particle and the nearest object in a particular direction we wish to check. This is intended to approximate the robot’s theoretical odometry should it have the same location and orientation as the particle. We start with the map indices of the particles location, and incrementally increase the magntidue of a ray in the direction we wish to check by 1 (in map index units), and check where the ray lands for a square that is nonempty, stopping if we find such a square (see diagram below).
* `update_particles_with_measurement_model()`: This function updates particle weights based on differences between the robot’s odometry and the particle’s theoretical odometry according to the above function. The weight given to a particle is the inverse of the sum of the absolute values of the differences between the particle measurement and the lidar measurement; large differences for directions that the robot would read as infinitely far away by capping both the theoertical and lidar measurement at the max radar range (3.5).
* ![image](https://user-images.githubusercontent.com/63179479/116186463-521aa200-a6e9-11eb-9ac4-2a4732861289.png)


#### Resampling
##### Code Location
The functionality of this portion is implemnted mainly in the two functions below. 
* `draw_random_sample()`: Line 38
* `resample_particles()`: Line 259
##### Functions/Code Description
* `draw_random_sample()`: This helper function receives a list of choices, their probabilities, and the number of samples to draw. For every sample to draw, we use `random_sample()` generate a random number between 0 and 1. To turn this into a choice from our list, we precompute the partial sums of the given probabiltiies, and then perform a binary search to find the interval among these partial sums which the value from `random_sample()` lies in. The _ith_ interval will correspond to the _ith_ choice, as the interval has length of the probability of the choice being chosen.
* `resample_particles()`: Resamples the particle cloud by calling `draw_random_sample()` on the the cloud and its weights, drawing the necessary number of particles we would like.

#### Incorporation of Noise
This functionality lies entirely within the function below at the specified lines.
##### Code Location
* `update_particles_with_motion_model()`: Lines 410-420
##### Functions/Code Description
* `update_particles_with_motion_model()`: Within this function, a helper function provided by numpy is used to randomly sample Gaussian distributions of specified sizes. We add on a 2d guassian vector with standard deviation 0.07 (mean 0) to randomly perturb the particles updated position, a random 1d guassian variable with a standard deviation of almost 4 degrees (0.68 radians) (mean 0) to randomly perturb the particles updated direction.

#### Updating Estimated Robot Pose
This functionality lies entirely within the function below.
##### Code Location
* `update_estimated_robot_pose()`: Line 335
##### Functions/Code Description
* `update_estimated_robot_pose()`: This funciton takes the average of the position coordinates and orientation quarternions of all the particles in the cloud to update the estimated robot pose.

#### Optimization of Parameters
The variables below are paramters that we experimented with to get the particle filter working. We didn't experiment with `self.lin_mvmt_threshold` or `self.ang_mvmt_threshold`, as this didn't occur to us, though it could have been helpful, and we found the gaussian noise to be just right on the first try.
##### Code Location
* `self.num_particles`: Line 107
* `self.directions_to_check`: Line 122
* `step_size`: Line 378
##### Functions/Code Description
* `self.num_particles`: Controls the number of particles
* `self.directions_to_check`: A list of all the directions (in radians) we want to compare the lidar scans of. More directions gives a better sense of which particles are good, but means more to check in every cloud update.
* `step_size`: The steps we take when casting a ray out from a particle to check for the nearest object in a direction. Larger steps means less accuracy for object distance, but quicker computation.


### Challenges
There were quite a few bugs, and the diagnosis for some of them tended to be quite misleading.  One issues occurred during the implementation of the measurement model. During testing, a strange divide by zero issue was causing issues. Although the source was initially difficult to discover, eventually we found that it was caused by not checking for infinite distance from Lidar scans, which is what occurs when there is no object in the given direction. Another bug resulted in the particles converging to exactly one point, and it would take massive leaps around the map. This was caused by not deep copying during resampling, but lead me (Carlos) to believe there were errors in the movement model (which there also were). There was also an error in the measurement model caused by a confusion between angles and radians ( and a decision to switch from using one to the othe).


### Future Work
If we had more time to work on this project, we would like to make our program more robust. We could lower the number of particles, so that the robot itself could move at faster speeds, but then we would be less confident that the particles would converge to the correct position. Thus it would be necessary to process the particle updates and resampling more quickly, so that either more particles can be used or a higher robot speed can be achieved. Another way would be to implement an entirely new model to handle robot localization, such as the beam model.

### Takeaways
* Divide the project into separate components and set specific deadlines for each component so that partners can work on the same timeframe.  We found that we preferred to work separately on our own time.  However, we did not set specific deadlines for components, leading to difficulty planning things since the components of this project must be done sequentially.
* Rigirously Debug each component separately as it is finished according to the implementation plan. Although we set a decent testing plan in implementation, in practice we delayed on testing at times and it led to greater difficulty debugging. Some of the testing I (Carlos) did initially did not catch bugs that caused me headaches later, because frankly it not as robust as it should have been. Making sure the measurement model behaved exactly as it should have would have definitely helped for example.
