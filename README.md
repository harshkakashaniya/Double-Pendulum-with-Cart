# Double-Pendulum-with-Cart
Designing a controller for stabilization of pendulums to its equilibrium state with its animation.

## Pendulum System.

<p align="center">
  <img width="300" height="350" src="Pendulum_gif.gif">
</p>

<p align="center">Animation of Controlled motion.
<p align="center">

# Steps of Project

## Made non-Linear system
Using Euler-Lagrangian equation to find a non-linear system of the given configuration.
## Linearized about equilibrium point
Then the system was linearized about the equilibrium point.  So that we can use the knowledge of linear system controller and can control system locally around the equilibrium point.

## Designed LQR system by choosing Q and R matrix for linear and non-linear system.
Choosing Q and R matrix big because given values of mass and lenght were very large and to change the dynamics we need such capable controller.

Linear system stabilized.

<p align="center">
  <img width="500" height="350" src="Images/LQR1.png">
</p>

<p align="center">Stability of first pendulum with cart.
<p align="center">
  
<p align="center">
  <img width="500" height="350" src="Images/LQR2.png">
</p>

<p align="center">Stability of Second pendulum with cart.
<p align="center">

### For non linear systems


<p align="center">
  <img width="500" height="350" src="Images/LQR_1.PNG">
</p>

<p align="center">Simulink system for nonlinear control
<p align="center">
  
<p align="center">
  <img width="500" height="350" src="Images/LQR_2.png">
</p>

<p align="center">Internal structure of LQR of non-linear
<p align="center">

## Designed Luenberger Observer for linear and non-linear system .

<p align="center">
  <img width="500" height="350" src="Images/L_observer.png">
</p>

<p align="center">Output of observer for linear system.
  
<p align="center">

<p align="center">
  <img width="500" height="350" src="Images/Luenberge_obs.PNG">
</p>

<p align="center">Simulink system for nonlinear observer.
  
<p align="center">



## Designed LQG for linear and non-linear system . 

<p align="center">
  <img width="500" height="350" src="Images/LQG_simulink.PNG">
</p>

<p align="center">Simulink system for nonlinear LQG controller.
  
<p align="center">

Note: If details required refer to the following report.

[Report](Robot_Controls_Final_project.pdf)
