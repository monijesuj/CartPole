# CartPole

## Overview

This project demonstrates several control strategies applied to the classic Cart-Pole balancing problem. Each approach is implemented in a separate file using different methodologies:

- **Linear Quadratic Regulator (LQR)** implemented in MATLAB.
- **Model Predictive Control (MPC)** implemented in a Jupyter Notebook.
- **PID and Cascade PID Control** implemented in a Jupyter Notebook.
- **Reinforcement Learning (RL)** using Proximal Policy Optimization (PPO) implemented in a Jupyter Notebook.

## File Structure

- **JAMES_LQR_final.m**  
  Implements the LQR approach by first establishing a state-space model for the cart-pole system. The script then discretizes the model, computes an optimal controller using MATLAB’s `lqr` function, simulates the closed-loop response, and finally animates the system dynamics.  
  See [JAMES_LQR_final.m](JAMES_LQR_final.m) for details.

- **JAMES_MPC_final.ipynb**  
  Contains a Model Predictive Control (MPC) solution that formulates the cart-pole problem as a convex optimization problem. This notebook leverages the `cvxpy` library to set up the optimization and uses the OSQP solver to optimize control inputs over a receding horizon. The resulting control actions are applied in simulation to stabilize the system.  
  Refer to [JAMES_MPC_final.ipynb](JAMES_MPC_final.ipynb) for the MPC implementation.

- **JAMES_PID_final.ipynb**  
  Provides simulations using both a standard PID controller and a cascade PID controller. The notebook integrates ODE solvers (via `solve_ivp`) to simulate the cart-pole dynamics under each control strategy. It then plots time-series data (cart position, pole angle, etc.) and creates animations to visualize system responses.  
  Check out [JAMES_PID_final.ipynb](JAMES_PID_final.ipynb) for the PID control methods and visualizations.

- **JAMES_RL_final.ipynb**  
  Uses Reinforcement Learning with the PPO algorithm from Stable Baselines 3. In this notebook, the cart-pole environment is defined (often leveraging gymnasium), and an RL agent is trained over multiple episodes. The performance of the trained model is evaluated through episodic metrics and visualized with plots and animations.  
  See [JAMES_RL_final.ipynb](JAMES_RL_final.ipynb) for the RL approach.

## Setup & Usage

### MATLAB (LQR)
- Open `JAMES_LQR_final.m` in MATLAB.
- The script works by:
  - Defining a state-space model of the cart-pole.
  - Discretizing the system for digital control.
  - Designing an optimal LQR controller using MATLAB’s built-in functions.
  - Simulating the closed-loop response and animating the cart-pole dynamics.
- Run the script to view the simulation.

### Python Notebooks (MPC, PID, RL)

#### Prerequisites
Ensure you have installed the following packages:
- `gymnasium`
- `numpy`
- `matplotlib`
- `cvxpy`
- `osqp`
- `scipy`
- `stable-baselines3`

#### General Workflow
1. **Open Notebook**: Open the respective `.ipynb` file in Jupyter Notebook or Visual Studio Code’s interactive environment.
2. **Sequential Execution**: Execute the cells one-by-one to run the simulations, evaluate models, and generate visualizations.
  
#### Specific Details

- **MPC Notebook (`JAMES_MPC_final.ipynb`)**:
  - Sets up the cart-pole model and defines an optimization problem.
  - Uses `cvxpy` to build the objective and constraints over a prediction horizon.
  - Solves the optimization problem with the OSQP solver to compute control actions at each time step.
  - Updates the system state based on these control actions and visualizes the results.

- **PID Notebook (`JAMES_PID_final.ipynb`)**:
  - Implements two control schemes:
    - **Standard PID**: Applies proportional, integral, and derivative control directly to the error signal.
    - **Cascade PID**: Uses a two-loop configuration where one PID controller regulates an inner loop (e.g., velocity) and another handles an outer loop (e.g., position or angle).
  - Simulates the dynamics of the cart-pole using numerical ODE solvers (`solve_ivp`).
  - Generates plots for key state variables and creates animations of the cart-pole motion.

- **Reinforcement Learning Notebook (`JAMES_RL_final.ipynb`)**:
  - Defines the cart-pole environment, possibly by extending or using an existing Gym environment.
  - Sets up and trains a PPO agent from the Stable Baselines 3 library.
  - Monitors learning progress over episodes, and evaluates performance using various metrics.
  - Visualizes learning curves and animates the controlled cart-pole in both training and evaluation phases.

## Results & Visualizations

Each implementation provides:
- **Time-series Plots:** For key variables such as cart position, velocity, pole angle, and angular velocity.
- **System Animations:** Both real-time or recorded animations depict the cart-pole dynamics.
- **Performance Metrics:** Detailed comparisons between different control strategies (e.g., evaluating standard PID versus cascade PID as well as the RL-trained agent) are provided.

## Conclusion

This repository serves as an attempt at the study of various control techniques for the cart-pole problem. 

Happy experimenting!