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


![architecture](/images/2.png)

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
