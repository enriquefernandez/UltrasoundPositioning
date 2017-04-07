# Ultrasound Positioning System using the Kalman Filter (by Enrique Fernandez)

Final project for MIT course 16.322 Stochastic Estimation and Control (Fall 2013).

The project report can be read [here](EnriqueFernandez_16322Project.pdf).

## How to run the code that generates the project plots

1. Open MATLAB and set the Working Directory to project_code/matlab
2. Run the run_16322_Project.m script and follow instructions
3. The plots will be generated in the graphs/ folder

## Arduino and data capture code

The code that operates the hardware (Arduino microcontroller) is placed in the Arduino folder.

The instructions to capture data (if an Arduino and the ultrasound hardware were connected would be):

1. Connect the Arduino to the computer
2. Open the arduino sketch and upload to the Arduino
3. Open MATLAB and execute the script matlab_read_direct_werrors.m inside the matlab/read_data folder
4. MATLAB and the Arduino are now recording data. Move the ultrasound emitter as desired for 10 seconds. (MATLAB will save 200 points at 20 Hz)
5. In order to save the data just captured, run the script save_meas_data.m

## Other interesting files

* __trilateration3d_EKF.m__ and __trilateration3d_UKF.m__ are the functions that implement the EKF and UKF respectively.
* __generate_results.m__ and __generate_noise_plots.m__ are the functions called by run_16322_Project.m to generate the plots
* __plot_loc_data.m__ generates a plot with the estimates in the xy plane and in the x, y and z axes independently, using the simple estimator, the EKF and the UKF
* __simple_estimate.m__ implements the simple estimator described in the project report.

## Contact information

Enrique Fernández
December 8, 2013

Massachusetts Institute of Technology, Cambridge, MA.
