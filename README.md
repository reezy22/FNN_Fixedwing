## Fuzzy Neural Network (FNN) Control System

This repository contains a MATLAB implementation of a **Fuzzy Neural Network (FNN)** based control system designed to stabilize dynamic systems, particularly in controlling the \(\theta\) (pitch) and \(z\) axes. The code implements various stability enhancements to reduce oscillations and ensure smooth control over time.

## Features

- **Fuzzy Logic Control**: Uses fuzzy membership functions to handle non-linearity in dynamic systems.
- **Neural Network Learning**: Adapts control parameters through learning rates to enhance system response.
- **Error Handling and Stability**: Includes safeguards against singularities and oscillations in the control response.
- **Configurable Parameters**: Allows users to adjust learning rates, membership functions, and other critical parameters to suit different control environments.

## Files

- `FNN_Controller.m`: Main MATLAB file containing the implementation of the FNN control system.
- `README.md`: Project description and documentation (this file).

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/username/FNN_Control_System.git
   ```
   
2. Open `FNN_Controller.m` in MATLAB.

3. Run the script to simulate the control system over a specified time period and visualize the control performance:
   ```matlab
   run('FNN_Controller.m')
   ```

4. Review the generated plots to analyze the control signals and system stability.

## Parameters

- **Learning Rates**:
  - `init_alpha_u`: Learning rate for \(u\) control signal.
  - `init_alpha_pitch`: Learning rate for pitch (\(\theta\)) control.
  - `init_alpha_z`: Learning rate for \(z\)-axis control.
  
- **Control Gain Matrices**:
  - `init_U_u`, `init_U_pitch`, `init_U_z`: Initial control gains for each axis.

- **Fuzzy Logic Parameters**:
  - `c_u_flc`, `c_pitch_flc`, `c_z_flc`: Centers for fuzzy membership functions.
  - `sigma_u_flc`, `sigma_pitch_flc`, `sigma_z_flc`: Sigma values to control the width of membership functions.

## Contribution

Feel free to submit issues or pull requests if you would like to contribute to the project or have suggestions for improvements.

## Author

- Arinze Ibemorah  - [@reezy22](https://github.com/reezy22)

