## Self Driving Car - Term 2 Project - Extended Kalman Filter

This project aims at developing an extended kalman filter for tracking
a moving object. In this case, a bicycle moving around a vehicle.
It uses Term 2 simulator to provide the lidar and radar measurements.
The flow is:
- simulator provides the measured lidar and radar measurement data
to the Filter
- the filter in turn does the processing on that data to calculate
the estimation marker as well as RMSE(Root Mean Square Error) value.

## Rubric Points

### Compilation

Code can be compiled using dockers on Windows. Docker settings are same as explained
in course material.

Ensure the following docker image can be seen in 'docker images' output:
- udacity/controls_kit

The following command can be used to compile the source code inside docker:
-  docker run -it -p 4567:4567 -v <FullPathOfFilterCodeOnWindows>:/work udacity/controls_kit:latest

- Once you are inside the docker, the cmake and make can be used to compile:

- mkdir _build
- cd _build
- cmake ..
- make

### To run

- Run kalman filter inside the docker
  - ./ExtendedKF
- Run the installed simulator from the host Windows OS
  - Select the option 'Project 1/2 EKF and UKF'
  - Press 'Start' button

### Output

- An output file gets generated with name 'output.txt' in the location where the binary is run.
This file has the input values(radar/lidar measurements) as well as RMSE values corresponding to those measurements.
- Also the green dots in the simulator represent the estimated values.   

### Accuracy

Last 5 RMSE values, taken up from the output.txt are:

rmse_x:  0.0974975 rmse_y:  0.0857352 rmse_vx: 0.452644 rmse_vy: 0.441595
rmse_x:  0.0976057 rmse_y:  0.0857164 rmse_vx: 0.452609 rmse_vy: 0.441248
rmse_x:  0.0975115 rmse_y:  0.0856301 rmse_vx: 0.452158 rmse_vy: 0.440805
rmse_x:  0.0974149 rmse_y:  0.0855442 rmse_vx: 0.45171 rmse_vy: 0.440374
rmse_x:  0.0973178 rmse_y:  0.0854597 rmse_vx: 0.451257 rmse_vy: 0.439941

As we can see these are within the acceptable limits of:
   [.11, .11, 0.52, 0.52]

### Flow of the algorithm

1. The first measurement is used to initialize the fields like timestamp and px and py values by the fusion filter.
2. The timestamp is stored in units of seconds.
3. From second measurement onwards, we first do the prediction step and then the update step.
4. In the update step, if the measurement type is of RADAR, we calculate and use the jacobian.
