# particle_filter_localization_project
![gif](particle_filter.gif)

### Objectives Description
The objective of this project is to apply our understanding of the particle filter algorithm to solve the problem of robot localization.  It is critical for a robot to understand its position relative to its environment in order to function properly.  The particle filter algorithm is a way to accurately estimate its location in a mapped environment.

### High-Level Description
We solved the problem of robot localization by implementing the particle filter algorithm shown in our lectures and coursework.  In our code, there are seven components of our approach:
1. `Initialization of Particle Cloud`: Initialize a cloud of randomized particles in mapped environment
2. `Movement Model`: Updating particle poses based on movements of robot
3. `Measurement Model`: Compare robot lidar scans with particles’ theoretical scans to obtain particle weights
4. `Resampling`: Normalize particles by weight and resample particles to converge on robot location
5. `Incorporation of Noise`: Factoring movement noise into movement model to account for real-world noise
6. `Updating Estimated Robot Pose`: Updating estimated robot pose based on particle field
7. `Optimization of Parameters`: Optimizing parameters to handle localization more efficiently

### Step Descriptions
#### Initialization of Particle Cloud
##### Code Location
* `gen_random_particle()`: Line 184
* `normalize_particles()`: Line 224
* `initialize_particle_cloud()`: Line 211
##### Functions/Code Description
* `gen_random_particle()`: This helper function generates a particle with a random location and orientation within the given environment.
* `normalize_particles()`: This helper function normalizes particle weights after they are all generated.
* `initialize_particle_cloud()`: This function generates a set number of randomized particles, normalizes their weights, and publishes them.

#### Movement Model
##### Code Location
* `get_yaw_from_pose()`: Line 25
* `update_particles_with_motion_model()`: Line 402
##### Functions/Code Description
* `get_yaw_from_pose()` : This helper function, given by the project administrators, is used to help calculate the robot’s change in orientation.
* `update_particles_with_motion_model()`: This function uses the robot’s odometry to find the robot’s change in position and orientation.  Then, it uses this data to update all particle poses.  Movement noise is accounted for within this model.  

#### Measurement Model
##### Code Location
* `get_yaw_from_pose()`: Line 25
* `estimate_particle_lidar()`: Line 370
* `update_particles_with_measurement_model()`: Line 393
##### Functions/Code Description
* `get_yaw_from_pose()`: This helper function, given by the project administrators, is used to help calculate the distance between particles and objects.
* `estimate_particle_lidar()`: This helper function is used to estimate distances between a particle and objects in all directions.  This is intended to approximate the robot’s theoretical odometry should it have the same location and orientation as the particle.
* `update_particles_with_measurement_model()`: This function updates particle weights based on differences between the robot’s odometry and the particle’s theoretical odometry.

#### Resampling
##### Code Location
* `draw_random_sample()`: Line 38
* `resample_particles()`: Line 259
##### Functions/Code Description
* `draw_random_sample()`: This helper function receives a list of choices, their probabilities, and the number of samples to draw.  Then, it repeatedly calls `random_sample()`, provided by numpy, to generate random numbers between 0 and 1.  These random numbers are used to choose particles from the original list of choices to add to the final set.
* `resample_particles()`: Updates the particle cloud by calling `draw_random_sample()` on the particle cloud, its particle weights, and the number of particles in the cloud.

#### Incorporation of Noise
##### Code Location
* `update_particles_with_motion_model()`: Lines 410-422
##### Functions/Code Description
* `update_particles_with_motion_model()`: Within this function, a helper function provided by numpy is used to randomly sample Gaussian distributions of specified sizes.  These random numbers are used in the calculation of the particles’ new estimated location.

#### Updating Estimated Robot Pose
##### Code Location
* `update_estimated_robot_pose()`: Line 335
##### Functions/Code Description
* `update_estimated_robot_pose()`: This function sums and averages the position and orientation values of all particles in the cloud to update the estimated robot pose.

#### Optimization of Parameters
TODO
##### Code Location
TODO
##### Functions/Code Description
TODO

### Challenges
Overall, we found that the actual execution of the project did not differ significantly from what we had outlined in our implementation plan.  As a result, the programming and debugging process went smoothly as a whole.  One of the biggest issues occurred during the implementation of the measurement model.  During testing, a strange divide by zero issue was causing issues.  Although the source was initially difficult to discover, eventually we found that it was caused by not checking for infinite distance from Lidar scans, which is what occurs when there is no object in the given direction.  Outside of this, most of the programming went well.


### Future Work
If we had more time to work on this project, we would like to speed up the convergence of the particles.  In the real world, it is not enough to simply create a working model, but one must also ensure that it is fast enough so that it can be applied in real-world situations.  This could be done a few ways.  One way would be improving the particle filter model we have already created.  Another way would be to implement an entirely new model to handle robot localization, such as the beam model.  Regardless, there would be many worthwhile directions we could take to improve our model further.

### Takeaways
* Divide the project into separate components and set specific deadlines for each component so that partners can work on the same timeframe.  We found that we preferred to work separately on our own time.  However, we did not set specific deadlines for components, leading to difficulty planning things since the components of this project must be done sequentially.
* Debug each component separately as it is finished according to the implementation plan.  Although we set a decent testing plan in implementation, in practice we delayed on testing at times and it led to greater difficulty debugging.
