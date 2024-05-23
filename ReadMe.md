# Inverse Kinematics of 7 DOF manipulator

This project develops an advanced inverse kinematics solution for the Maira robot using Denavit-Hartenberg (DH) parameters. It calculates orientation errors with quaternions, enforces joint limits (±180° for all axes, except A2: ±120° and A4: ±150°), computes the Jacobian matrix, and handles redundancy using pseudo-inverse, null-space projection, and a secondary objective enforcing the joints to stay close to mean of its ranges. The implementation is object-oriented, efficient, and includes condition number calculation for the Jacobian. A test function estimates the success rate in terms of Forbenius norm of the difference of target and converged transformations, that encapsulates the proximity of the solution.  

## Table of Contents

- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Building the Project](#building-the-project)
- [Testing the Project](#testing-the-project)
- [Running the Project](#running-the-project)
- [Solution Approach](#solution-approach)

## Prerequisites

Before you begin, ensure you have met the following requirements:

- You have installed a C++ compiler (e.g., g++, clang++)
- You have installed [CMake](https://cmake.org/download/)
- You have installed [Make](https://www.gnu.org/software/make/)
- Additional dependencies of Eigen and Gtest will be added through the `CMakeLists.txt`.

## Project Structure

Open up your command line and first clone the repository and change the directory to the project. 
```sh
git clone https://github.com/Shubham1965/InverseKinematics7dof.git
cd InverseKinematics7dof
```

You'll see the following directory structure.

```plaintext
InverseKinemtics7dof/
├── include/
│   └── ... # header files
├── src/
│   └── ... # source files
├── tests/
│   └── ... # test files
├── CMakeLists.txt
└── README.md
```

## Building the project
In order to build the project, execute the following commands. 
```sh
mkdir build
cd build
cmake ..
make
```

After building, the directory structure should look like this:

```
yourproject/
├── include/
│   └── ... # header files
├── src/
│   └── ... # source files
├── tests/
│   └── ... # test files
├── build/
│   ├── src/
│   │   └── bin_main.exe
│   ├── tests/
│   │   └── test_ik.exe
│   └── ... # other build files
├── CMakeLists.txt
└── README.md
```


## Testing the project
Before running the project, it is important to test whether all functiions work. Ideally there should have been tests for each fucntioin definition but here i build it jut for one. So, in order to test the project, navigate to the `build/tests` directory and execute
```sh
./tests/test_ik.exe
```
This is a test routine to check whether `InverseKinematics::solve` function always works and gives an output. This is just a sanity check. Ideally, all tests should pass.

## Running the project:
To run the main executable after testing, navigate to the `build/src` directory and execute
```sh
./src/bin_main.exe
```
This is the main test implementation where you can change the desired position and orientation, 
```c++
// Target position and orientation
// Example target position describing the target position is at (0.5m, 0.3m, 0.6m) in the 3D space
Eigen::Vector3d target_position(1.0, 0.3, 0.6); 

// Example quaternion (w, x, y, z) describing the target orientation as a 90-degree rotation around the z-axis
Eigen::Quaterniond target_orientation(-0.7071, 0, 0, 0.7071); 

```
and the initial configuration,

```c++
Eigen::VectorXd initial_theta(7);
initial_theta << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // Zero initial joint angles
```
and rebuild, test and run in order to check the success rate of convergence within 50 iterations.


## Solution Approach

The main crux of solving inverse kinematics numerically is handling the redundancy when you have more number of degrees of freedom than the dimensionality of the operational space. By incorporating the following  modifications, you can handle redundancy more effectively and ensure that the joint limits are respected during convergence.

- **Damped Pseudo-Inverse**: A damping factor is added to the pseudo-inverse calculation to improve numerical stability, especially to avoid singularities.

- **Null-Space Projection**: The null-space matrix 𝑁 allows for movements that do not affect the primary task. This is used to optimize secondary objectives, such as keeping joint angles away from limits.

- **Secondary Objective**: A simple gradient descent towards the middle of the joint range helps avoid joint limits. The secondary objective term xxxxx can be adjusted according to the specific requirements of your application.

- **Joint Limit Enforcement**: After each iteration, the joint angles are clamped to ensure they stay within the specified limits.


View the complete algorithm here - [Inverse Kinematics Algorithm ](doc/Inverse_Kinematics.pdf)

> Initial $q_{seed}$, the damping factor $\lambda$, and the secondary objective gain $\alpha$ plays a crucial role in convergence. They act as a parameter set that needs to be further tuned in order to converge within stipulated amount of iterations. Further refinement would be on how to change these parameters to find the optimal deviation on angles and quickly converge to the optimal configuration within the iteration limits. 
