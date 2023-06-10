# Vehicle Dynamics and Nonlinear Model Predictive Control Simulation with Constraints

## Description

This MATLAB script simulates vehicle dynamics and an associated nonlinear model predictive control (NMPC) algorithm while incorporating constraints on the vehicle's state and control inputs. 

The main function `vehicle_dynamic_model_6_states` calculates the vehicle dynamics given the current state of the vehicle and the control inputs. This function is used within the NMPC simulation to predict the vehicle's state trajectory.

The NMPC problem, which is a nonlinear programming problem (NLP), is formulated using the CasADi library. The cost function of the NLP aims to minimize the difference between the current state and the reference state, as well as minimize the control effort. Constraints are implemented to ensure that the vehicle states and control inputs stay within their specified bounds. 

The simulation loop at the end of the script advances through time, updating the NMPC problem at each time step, and applies the first control input from the NMPC solution to the vehicle dynamics model. The states and control inputs are stored for each time step.

## Dependencies

- MATLAB
- CasADi: A symbolic framework for automatic differentiation and numeric optimization. Download and installation guide [here](https://web.casadi.org/).

## File Structure
- `vehicle_dynamic_model_6_states.m` - Function file calculating the vehicle dynamics given the current state and the control inputs.

## Inputs

### System State
`x` (vector of length 6): 
- `x(1)`: Longitudinal speed (vx)
- `x(2)`: Lateral speed (vy)
- `x(3)`: Yaw rate (psi_dot)
- `x(4)`: Yaw angle (psi)
- `x(5)`: Longitudinal position (X)
- `x(6)`: Lateral position (Y)

### Control Inputs
`u` (vector of length 2): 
- `u(1)`: Longitudinal acceleration (ax)
- `u(2)`: Lateral acceleration (delta_f)

### Time Interval
`Ts` : Sampling time for the NMPC.

## Outputs

- The script outputs plots of the longitudinal speed (vx), lateral position (Y), longitudinal acceleration (ax), and lateral acceleration (delta_f) of the vehicle over time.
- It also outputs the total simulation runtime in the MATLAB console.

## Constraints

The NMPC problem imposes constraints on the vehicle states and control inputs. 

### State Constraints:
- Longitudinal speed (vx) is constrained between 0 and 30 m/s.
- Lateral speed (vy) is constrained between -5 and 5 m/s.
- Lateral position (Y) is constrained between -2 and 2 m.

### Control Input Constraints:
- Longitudinal acceleration (ax) is constrained between -2√2 and 2√2 m/s^2.
- Lateral acceleration (delta_f) is constrained between -π/18 and π/18 rad.

Additionally, constraints are imposed on the changing rate of ax and delta_f.

## Usage
1. Ensure that MATLAB and CasADi are correctly installed on your system.
2. Navigate to the directory containing the `.m` file in your MATLAB environment.
3. Run the script.

## Limitations
This code does not consider real-world constraints such as traffic or road conditions. It provides a rudimentary representation of vehicle dynamics and predictive control, intended primarily for educational and developmental purposes.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. Please make sure to update tests as appropriate.
