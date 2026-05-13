Simulation creation of the following paper: https://arxiv.org/pdf/2111.01302

Only implements the kinematics (no dynamics with torques and a controller)

To get started:

```
cd file/to/project/
```

run simulate.m

Some info:
Inside the robot_state, generateBodyRateFunction.m generates the derivative functions for calculating omega_b, the rate of rotation.

Given a generated CoM traj:

1. From the paper, we use equations 21-23 to recover the roll and pitch.
2. We also use equation 1 to compute the analytical derivative of the angular velocity.
3. From these results, we can recover the linear base velocity
4. Using ode45, we can calculate the linear base position