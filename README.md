# Control and Estimation of a Quadcopter

This repository contains the implementation of various controllers used to control a quadcopter to follow a trajectory. The implementation is done in MATLAB.

## LQR control

The LQR folder contains the code that implements a LQR control as well as a LQG control. To set up the variables for the quadcopter, run quadcopter_model then discretization. The following files implements different controllers:

lqr_tracking: LQR control with zero design

finitehorizonlqr_tracking: Finite horizon LQR control with zero design

lqr_integral: LQR control with Integral control

lqg_integral: LQG control with Integral control

## MPC

The MPC folder contains the code that implements Model Predictive Control. To set up the variables for the quadcopter, run quadcopter_model then discretization. The following files implements different controllers:

mpc_lqr_tracking: MPC using finite horizon LQR 

mpc_y_nonlinear: MPC using quadprog

mpc_with_estimator: MPC using quadprog with state estimation
