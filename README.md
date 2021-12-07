# Piston-Driven Pneumatically-Actuated Soft Robots: modeling and backstepping control

## Abstract
Actuators' dynamics have been so far mostly neglected when devising feedback controllers for continuum soft robots since the problem under the direct actuation hypothesis is already quite hard to solve. Directly considering actuation would have made the challenge too complex. However, these effects are, in practice, far from being negligible. The present work focuses on model-based control of piston-driven pneumatically-actuated soft robots. We propose a model of the relationship between the robot's state, the acting fluidic pressure, and the piston dynamics, which is agnostic to the chosen model for the soft system dynamics. We show that backstepping is applicable even if the feedback coupling of the outer on the inner subsystem is not linear. Thus, we introduce a general model-based control strategy based on backstepping for soft robots actuated by fluidic drive. As an example, we derive a specialized version for a robot with piecewise constant curvature.

## Paper
This repository contains the simulations as presented in the paper _Piston-Driven Pneumatically-Actuated Soft Robots: modeling and backstepping control_ by Maximilian Stölzle and Cosimo Della Santina published in the _IEEE Control Systems Letters_.

Please cite our paper if you use our method in your work:
```bibtex
@article{stoelzle2022piston,
  title={Piston-Driven Pneumatically-Actuated Soft Robots: modeling and backstepping control},
  author={Stölzle, Maximilian and Della Santina, Cosimo},
  journal={IEEE Robotics and Automation Letters},
  publisher={IEEE}
}
```

## Simulation

## Folder structure
The following folder structure is used:
- `main.slx`: contains the main simulink model which implements both the controller and the system model
- `system_model.slx`: contains the system model

## Usage
1. Run the `startup.m` file to set up the environment
2. Derive the dynamics for the planar soft robotic arm through the PCC assumption with the script `derive_dynamics.m`. The resulting symbolic functions for evaluating the dynamics at a specific robot state are automatically saved in the folder `funs`.
2. Generate a control reference sequence with the script `generate_control_ref_sequence.m`. The sequence is automatically saved in `data/in_qref_ts.mat`.
3. Run the simulink model `main.slx` to simulate the system.
4. Animate the PCC system using the script `plotting/animate_pcc_system.m`. It automatically uses the time-series data from the simulation which is saved in the `out` variable of the workspace.
5. Save the `out` variable in a mat-file and save it in the `data` folder.
6. Generate time-series and Cartesian evolution plots using the scripts `plot_cartesian_evolution.m` and `plot_time_series_v2.m` in the `plotting` folder.
