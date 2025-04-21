# 2D Drone Simulation

This project contains a basic 2D simulation of a drone using Python. 

The goal is to build a sandbox environment with different levels of modelling complexity, in order to test different control strategies and study the system dynamics.

## Features

- Models the translational and rotational dynamics of a 2D drone
- Simulates the effect of individual motor thrusts
- Includes a simple animation to visualize drone movement

## Equations of Motion

The drone is modeled using the following equations:

- Horizontal acceleration:
  $$
  \ddot{x} = -\frac{(F_1 + F_2) \cdot \sin(\theta)}{m}
  $$

- Vertical acceleration:
  $$
  \ddot{y} = \frac{(F_1 + F_2) \cdot \cos(\theta)}{m} - g
  $$

- Angular acceleration:
  $$
  \ddot{\theta} = \frac{(F_2 - F_1) \cdot L}{2I}
  $$

where:

- $ F_1 $, $ F_2 $: Thrust from left and right motors
- $ \theta $: Drone orientation
- $ L $: Distance between motors
- $ m $: Drone mass
- $ I $: Moment of inertia
- $ g $: Gravitational acceleration
