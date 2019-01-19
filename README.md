# Quadcopter-Simulation  

Code for controllers (PD, LQR) for flying quad-copter on various trajectories generated with way-points and dynamic constraints on MATLAB  

The project emphasizes the generation of optimal time-parameterized piecewise continuous trajectories
and feedback control design to enable an aerial robot (in simulation) to fly along a pre-defined path. The project
seeks to:

* Develop a basic state machine to facilitate simulation that enables the robot to takeoff, hover, track a
trajectory, and land;
* Introduce time-parameterized trajectories given fixed initial and final endpoint constraints and bounded
velocity and acceleration;
* Extend the formulation to include piecewise continuous trajectories with fixed initial and final endpoint
constraints;
* Evaluate tracking performance given different levels of flight performance and trajectory design (from slow
to fast, straight and curved paths)


![architecture](/media/2.png)

**Orange** – Initial conditions  
**Blue** – Feedback/ updation

We first initialize are trajectorydes and trajectoryact for t=0.  
We run the loop in small time steps and keep updating the linear and angular positions, velocities, accelerations.

**Format of both trajectorydes and trajectoryact:**
xyz , xyz_vel, xyz_acc, xyz_jerk, xyz_snap, rpy, rpy_vel, rpy_acc
size is 3x8xN
where N = number of small timesteps for the trajectory

**Names of the files for these main functions:**
PDinitialize.m, PDcontroller.m, position_controller.m, attitude_controller.m, motor_controller.m,
dyanamic_model.m

**Names of the files for fast calculation of roll pitch desired functions:**
rollpitchdes_eq.m, rollpitchdes_d_eq.m, rollpitchdes_dd_eq.m, attitude_controller_equation_generator.m

**Names of the files for simulating and plotting graphs:**
plot_graphs.m, plot_graphs_velocities.m, simulate_quadcopter.m


# TASKS:  
They are given in the tasks folder and can be run individually:


**2.Hover Performance:**    
Using the linearized feedback control policy, implement a PD
feedback controller to enable the robot to hover at a desired location (e.g., z = 0.5 m). Create a simulation
scenario where the robot transitions between multiple waypoints along the x-axis by specifying goals that
increment by 10 cm in the x direction. Plot the error between the desired pose (position and orientation) and the
actual pose. Modify the gains associated with the position control (outer loop) and attitude control (inner loop). Plot the response for multiple gains.

![2](/media/t2.gif)

**3. Line-tracking Performance:**      
Develop a PD line tracking controller to enable the robot to take-off from a
starting location, go to a fixed height of 1.0 m, and return to the ground. Plot the error between the desired pose (position and orientation) and the actual pose. Modify the gains associated with the position control (outer loop). Plot the
response for multiple gains.

![3](/media/t3.gif)


**4. State Machine:**     
Develop and detail a state machine that enables the platform to takeoff to a pre-specified
height, hover, track trajectories, and land. Provide a description of this state machine with an associated state
machine diagram. One possible strategy is to develop a finite sequence of modes that transition based on the
current and desired states as well as the prior mode.  
* The system begins in an idle state generating no (or null) control inputs.
* A simulation trial begins by transitioning into a takeoff state that consists of a trajectory tracking
controller from the current robot state to a desired hover state (fixed pose). When the robot approaches
the desired hover state (based on the error between current and desired pose), the robot transitions into
a hover mode.
* The platform remains in hover mode for a small amount of time (e.g., 5 s) before transitioning into a
tracking mode.
* The platform tracks a specified trajectory and upon completion transitions into the hover mode.
* After remaining in hover mode for a brief period of time, the robot transitions into land mode and
begins a descent to the ground.

![4](/media/t4.gif)

**5. Gain Selection and Tuning:**     
After commanding the robot to takeoff and hover, generate a command that
tracks a single waypoint at [0, 0, 0.1] m (zero velocity). Plot the error response of the system with respect to the
desired position, orientation, and linear and angular velocities. Measure the rise (90%) and settling (10%) times
associated with the position and linear velocities, the steady-state value, the maximum percent
overshoot. Now provide a waypoint at the same position but with heading of 15 deg and loof for the similar time
domain performance characteristics of the heading controller. Analyze different values (position and heading)
with given different gain values.

![5](/media/t5.gif)

**6. LQR Controller Design and Evaluation:**    
Repeat the evaluation in (2), (3), and (5) using an LQR-based
feedback controller. Observe the performance difference in terms of error response characteristics.

![6_2](/media/t6_2.gif)
![6_3](/media/t6_3.gif)
![6_5](/media/t6_5.gif)

**7. Bounded Acceleration Trajectory Generation:**     
Develop a straight line time-parameterized polynomial
trajectory with initial and final endpoint constraints ([0, 0, 1] and [0, 0, 10], respectively; higher-order terms
zero). Choose a time-scaling that ensures a bounded (maximum/minimum) acceleration less than 3 m/s2.
Generate a set of error plots that depict the performance of the platform when tracking the trajectory (using PD
control) including the error in the pose and linear/angular velocities. As you increase the desired acceleration bound, at what value does the system begin to exhibit degraded tracking accuracy? Observe how does the performance
improve by artificially increasing the motor gain configuration parameter thereby effectively upgrading the
robot motors.

![7](/media/t7.gif)

**8. Minimum Energy Elliptical Trajectory:**   
Define a piecewise continuous trajectory consisting of four
waypoints: [0, 0, 1, 0], [2, 1, 1, 0], [0, 2, 1, 0], [-2, 1, 1, 0] (position, heading). To do so, generate an optimal
(minimum energy) trajectory that visits the four waypoints (returning to the first; forming an ellipse) with an
initial velocity of zero and an end velocity of 1 m/s at the first waypoint. The robot will start at rest, track the
elliptical trajectory, and arrive at the starting location at a non-zero velocity (1 m/s). Generate a second
trajectory phase given the same waypoints that starts with a 1 m/s velocity tangent to the ellipse and similar
velocities at the other waypoints (all tangent to the ellipse in the direction of motion). To solve for the optimal
trajectory, formulate the problem as a Quadratic Project (QP) and solve for the appropriate polynomial
coefficients. Generate error plots that depict the tracking performance (using PD control). Generate a cumulative
error distribution plot. Observe how does the tracking performance change given differing velocity profiles.

![8_1](/media/t8_1.gif)

**9. Robot Pirouette:**   
Repeat the same steps as specified in (7) but now with heading commands that cause the
robot to point to the center of the ellipse (second trajectory phase). Observe how does the performance change when
introducing time-varying heading commands at high-speeds.

![9](/media/t9.gif)

**10. Enforcing Smooth State Machine Transitions :**    
The naive state machine proposed in (4) does not enforce the
requirement that feedback control references can exhibit non-smooth (and non-trivial) jumps in the desired pose,
velocities, and higher-order terms. As an example, consider the transition from the trajectory mode to the hover
mode based on an error term that only considers the desired and current pose (ignoring the non-zero velocity and
acceleration). Propose, implement, and evaluate two strategies to address and mitigate rapid changes in modes
by saturation or gain selection. Possible strategies may include bounding changes in the input reference via a
saturation function or choosing softer gains.

![10](/media/t10.gif)

