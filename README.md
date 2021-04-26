# particle_filter_localization_project

### Objectives Description
The objective of this project is to apply our understanding of the particle filter algorithm to solve the problem of robot localization.  It is critical for a robot to understand its position relative to its environment in order to properly function.  The particle filter algorithm is a way to accurately estimate its location in a mapped environment.

### High-Level Description
We solved the problem of robot localization by implementing the particle filter algorithm shown in our lectures and coursework.  In our code, there are seven components of our approach.  
1. `Initialization of Particle Cloud`: Initialize a cloud of randomized particles in mapped environment
2. ‘Movement Model’: Updating particle poses based on movements of robot
3. ‘Measurement Model’: Compare robot lidar scans with particles’ theoretical scans to obtain particle weights
4. ‘Resampling’: Normalize particles by weight and resample particles to converge on robot location
5. ‘Incorporation of Noise’: Factoring movement noise into movement model to account for real-world noise
6. ‘Updating Estimated Robot Pose’: Updating estimated robot pose based on particle field
7. ‘Optimization of Parameters’: Optimizing parameters to handle localization more efficiently
