# Introduction of ALTRO

ALTRO (Augmented Lagrangian TRajectory Optimizer), a very fast solver for constrained trajectory optimization problems. ALTRO uses iterative LQR (iLQR) with an augmented Lagrangian framework and can solve problems with nonlinear inequality and equality path constraints and nonlinear dynamics. The key features of the ALTRO solver are:

* General nonlinear cost functions, including minimum time problems
* General nonlinear state and input constraints
* Infeasible state initialization
* Square-root methods for improved numerical conditioning
* Active-set projection method for solution polishing

For details on the solver, see the original [conference paper](http://roboticexplorationlab.org/papers/altro-iros.pdf) or related [tutorial](https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf).

# About this repository

This is an port version of the C++ implementation of ALTRO (Originated from [bjack205/altro: A Fast Solver for Constrained Trajectory Optimization](https://github.com/bjack205/altro)). I've packaged some of the author's work and some of the necessary dependency libraries so that they can be deployed and used in ArduinoIDE with a single click.

This project is open source under the MIT protocol. If this repository infringes on someone's interests, please contact me immediately and I will remove it!

# Dependencies

The Arduino ALTRO library relies on customized libfmt for embedded device development environments, so I've packed it in the library without having to additionally download libfmt and deploy it.

| Name         | URL                                                                                                                               |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------- |
| ArduinoEigen | [hideakitai/ArduinoEigen: Eigen (a C++ template library for linear algebra) for Arduino](https://github.com/hideakitai/ArduinoEigen) |
| fmt-arduino  | [DarkWizarD24/fmt-arduino: Port of the {fmt} library to Arduino](https://github.com/DarkWizarD24/fmt-arduino)                        |

# Notes

This project has only been tested on the WeAct STM32H750VBT6 development board so far, which requires at least 150KB of storage space after compilation. So in order to avoid memory explosion, we need to use all the storage space of H750 (including 2MB reserved space), so please select WeAct STM32H743VIT6 when choosing “BoardType”, and uploading mode should be DFU mode, and uploading the program via USB cable.

Please use STM32CubeProgrammer to check the size of the chip yourself, if you have a chip with 200KB of program storage, then this is likely to be a problem!
