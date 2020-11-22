# The Quadrotor PID Cascaded 3D Controller #


## Development Environment Setup ##

For environment setting and build steps, please checkout the original [README](./README.md).


## Run The Tasks ##

First you need to run the simulator.


### Then in `scenario 1`:

I change the `QuadControlParams.Mass * 9.81 / 4` to `QuadControlParams.Mass * 9.81 / 5` in `QuadControlParams.txt`.


### Body rate and roll/pitch control (scenario 2) ###

1. Implement body rate control

 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
 - In `GenerateMotorCommands()` I just solved the equation 

    `tau_x / l = F_1 + F_4 - F_2 - F_3`

    `tau_y / l = F_1 + F_2 - F_3 - F_4`

    `tau_z / kappa = F_1 - F_2 + F_3 - F_4`

    `c_total = F_1 + F_2 + F_3 + F_4`

 - `BodyRateControl()` is a simple one that just calculate the acceleration and convert to moment.

2. Implement roll / pitch control

 - This function need a little tricky, it need to get `target_R13` and `target_R23` and then solve this one:
    
    $$
    \begin{pmatrix} p_c \\ q_c \\ \end{pmatrix}  = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c  \end{pmatrix} 
    $$

    
### Position/velocity and yaw angle control (scenario 3) ###

 - `LateralPositionControl()` is a typically a PD controller. But need to handle maximum acceleration.
 - `AltitudeControl()` calculate PD terms to get `u_1_bar`, and use $$c = (\bar{u}_1-g)/b^z$$  
    to get world frame acceleration. 
 - I don't think it's a good idea that (`kpVelXY` and `kpVelZ`) take 3-4 times greater than the (`kpPosXY` and `kpPosZ`).

    
### Non-idealities and robustness (scenario 4) ###

 - I add a integral term with `KiPosZ * integratedAltitudeError`, then the red one can fly to the goal(without a little shift back to the goal).
 
<p align="center">
<img src="animations/scenario_4_1.png" width="300"/>
<img src="animations/scenario_4_2.png" width="300"/>
</p>
 

### Extra Challenge 1 (Optional) ###

 - Change the initial value of `pz` from 0 to the nearest z value in the trajectory to avoid the first `vz` is too large.
 - I just add a coefficient to control the velocity.
 - After speed up the drone, I need to tune some params again to give more control.