# Max-Desparate-New-Fast-Robots-Site.github.io
I give up on a fancy website (too many hours lost over broken js files) and desparately try to get a new one going. 

# Lab 4
In this lab, I set up the IMU sensor, enabling 9DOF sensing for the Artemis board. This enables more complex path planning and systems control.

![alt text](lab4/imuconnect.jpg "Picture of the IMU connected to the Artemis board.")

The example code for the IMU board runs without issue out-of-the-box. (The below GIF was recorded retroactively with the example code. The IMU has already been mounted to the robot. The outputs are read from Serial plotter.)

![alt text](lab4/examplecode.gif "Showing the sample code track IMU data from the robot position")

The AD0_VAL value is set to zero by default. This variable refers to the LSB bit of the I2C address. Manipulating the AD0_VAL could allow up to two IMU boards to run in parallel. (Having two of the same board would improve robustness of sensor data.) 

## Accelerometer
Using the equations from class, I record pitch and roll values with the IMU board positioned at -90, 0, and 90 degrees in these two axes.

'''c
pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
'''

![alt text](lab4/pitch90090.png "Image of pitch data sampled at different angles.")

![alt text](lab4/roll90090.png "Image of roll data sampled at different angles.")

To get this data, I held the IMU board against the flattest surfaces I had in my room. This brings about potential systematic errors since the flatness and perpendicularity of these surfaces are not guaranteed. This affects my two-point calibration.  

The raw accelerometer data is very noisy. Additionally, at the extreme angles, data for the other axis is most inconsistent. While most of the issue of the accelerometer data is in precision, some of the accuracy is calibrated for by determining a shift and scale factor from the sampled data to fit the expected values to the expected angles. 

'''python
# Calculating conversion factors
# Scale conversion
s = 180 / (np.mean(pitch90) - np.mean(pitchneg90))
# origin conversion
o = -(np.mean(pitch90) + np.mean(pitchneg90))/2
print(s,o)
>> 1.0259896646740851 1.0943006994729316
'''

