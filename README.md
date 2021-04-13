# particle_filter_localization_project

**Team Members**
Carlos Azpurua, Michael Su

**initialize_particle_cloud**

We will initialize a large number of particles with random x and y coordinates within the map.  They will also have a randomly generated orientation between 0 and 360 degrees.  
 
To test this, we will print some of the particle data after initialization to ensure that the particle variables are within expected ranges.

**update_particles_with_motion_model**

We will subscribe to the odometry topic to get the velocity of the robot, and so every time we receive a message from this topic, we can update the particle cloud by moving the particles by the velocity times the time between the last update from odometry (or less frequently if necessary). 
 
To test this, we will generate a particle on the robot, and see how closely the particle is able to follow the robot over time.

**update_particle_weights_with_motion_model**

For every particle, we will use the occupancy grid to obtain eight distances by searching in eight equally spaced directions. We compare this with the Lidar Scans at the same angles (which will depend on the orientation of the robot), and then get a weight by taking the inverse of the sum of the differences between lidar scans.
 
To test this, we will generate a particle on the robot, check if its weight is large, and a particle off far from the robot and check if its weight is relatively small.


**normalize_particles & resample_particles**

We will sum the weights of all the particles, and divide the weight of each particle by this sum to obtain the normalized weight. Then we assign each particle a sub-interval with length of the normalized weight in the interval [0,1]. To sample a particle, we randomly generate a value in the interval [0,1] and select the particle corresponding to the interval that this value lies in.

To test normalize_particles, we will create a small set of particles that can easily be checked for importance weights by hand. Then, we will normalize their weights by hand and compare it to the results of the normalize_particles function to check for correctness.  To test resample_particles, we will create a small set of particles with distinct importance weights (e.g. [.5, .3, .1, .1]).  Then, we will use resample_particles to generate 50 or so resampled particles and manually inspect the resampled particles to see if they were generated based on the test set’s importance weights.

**update_estimated_robot_pose**

To estimate the pose of the robot, we will take the average x and y coordinates of all the particles, respectively, and then take the average angle of each particle.

To test this, we will manually create a few small sets of particles so that we can manually calculate their average x, y, and angle values.  Then, we will test those same sets of particles on this function to see if we obtain the same result.

**Incorporate Noise**

We will sample the gaussian distribution to randomly perturb both the position and direction of every particle by adding said distribution to the position and direction.


We will test this by observing the perturbation on some fixed set of particles, to make sure the perturbations are not too extreme. During integration testing we will also note whether the particles are able to more consistently converge with noise, and whether the noise slows down the particles from converging.

**Timeline**

Our goal is to have the first three functions completed by 4/18, and then the final three functions completed by 4/23 to have some room for integration testing.
