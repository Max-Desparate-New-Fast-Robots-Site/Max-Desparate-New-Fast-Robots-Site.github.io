# Max-Desparate-New-Fast-Robots-Site.github.io
I give up on a fancy website (too many hours lost over broken js files) and desparately try to get a new one going. 

# Lab 4
<details>
In this lab, I set up the IMU sensor, enabling 9DOF sensing for the Artemis board. This enables more complex path planning and systems control.

![alt text](lab4/imuconnected.jpg "Picture of the IMU connected to the Artemis board.")

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

```python
# Calculating conversion factors
# Scale conversion
s = 180 / (np.mean(pitch90) - np.mean(pitchneg90))
# origin conversion
o = -(np.mean(pitch90) + np.mean(pitchneg90))/2
print(s,o)
>> 1.0259896646740851 1.0943006994729316
```

Using a fourier transform, I derived an alpha term for low-pass filtering. 
![alt text](lab4/pitchfft.png "Pre-filtered Pitch FFT")

I decided to use 2Hz as my cutoff frequency, leading to the following calculations. 
![alt text](lab4/alphacalc.png "Alpha calculation")

This leads to a much smoother fourier transform for the filtered signal. In fact, for the low-pass filtered pitch data, only three peaks are registered by the FFT.
![alt text](lab4/pitchlpffft.png "Post-filter Pitch FFT")

Here are the pitches laid on top of one-another. The filtered pitch is clearly much smoother and robust.
![alt text](lab4/pitchlpfcompare.png "Pre- and post-filter Pitch FFT")


## Gyroscope
The gyroscope data suffers from clear drift over time. This is inherent to the sensor, although I found tha higher sampling rates reduced this drift. I suspect this has to do with the dt calculation involved with iterating pitch, roll, and yaw values from gyro data. 

![alt text](lab4/gyrodrift.png "Gyro pitch drift over 5 seconds")

I merged the gyro data with the accelerometer data using a complimentary filter for higher accuracy and precision, with robustness to noise and rapid changes.

I used a beta of 0.9 for these results.

![alt text](lab4/pitchreadingcompare.png "Pitch complimentary")

![alt text](lab4/rollreadingcompare.png "Roll complimentary")

## Sample Data and Stunts
The limiting factor for my sampling rate was waiting on the TOF sensor data. The change I made to boost my sampling rate substantially was to not wait for TOF sensor data when it was not available, and update IMU data first. I used separate float arrays for my variables of interest with a set size of 1400, experimentally determined to be more than enough for 5 seconds of data. 

My sampling rate was much faster when I spent the 5 seconds filling up my data buffers and incrementally sending the data after the recording process was complete. The downside of this setup is that the real runtime is much longer than 5 seconds. 

![alt text](lab4/5secreadings.png "5 seconds of data")

I cut the cord for the 850mAH battery, the smaller voltage of the two supplied batteries, to be connected to the Artemis. The larger voltage battery is more suited for the high-power demands of the motor controllers. 

I did some basic stunts, with and without the Artemis onboard. 

Without Artemis:
![alt text](lab4/noart.gif "Test drive without Artemis")

With Artemis:
![alt text](lab4/art.gif "Test drive with Artemis")


The following data is from a 5 second capture from the IMU and TOF sensor onboard the robot, run remotely. This particular scene shows me driving the robot 360 degrees in my messy room. Notice the yaw values responding appropriately. Pitch and roll appear to jitter likely due to friction interactions with the floor. Distance sensing also responds appropriately. (I have TOF sensors on either end of the robot.)

![alt text](lab4/360spindata.png "Data from a 360 spin")

</details>

# Lab 5
<details>

## Wiring Diagram
![alt text](lab5/diagram.jpg "Motor driver wiring diagram")
The Artemis voltage requirements for powering are different than that of the motors. The motors draw signficantly more power than the computer. Therefore, a lower voltage battery is suited for running the Artemis computer and a higher voltage batter is suited for running the motors. 

I use analog pins on opposite sides of the Artemis board to control the motor drivers. This makes debugging easier due to the intuitive visual grouping and also reduces likelihood of me shorting connections during the soldering process. 

## Power Supply Testing
A control signal amplitude of 2V best matches the expected output voltage from the Artemis board. For Vin, I used 3.7V to match the maximum output of the charged 850mAh battery. 

I was successfully able to module the speed of the motors on each side using the duty cycle from the function generator. Shorter duty cycles lead to slower rotation.
![alt text](lab5/labsetup.gif "Robot motors running tethered")

## Calibration
I added the following code to the bluetooth cases to allow for easy testing and calibration of left and right motors. For a left motor and right motor speed sent over bluetooth, the robot moves forward at those speeds for a second.

```C
case TEST_DRIVE:
    success = robot_cmd.get_next_value(l_speed);
    if (!success)
        return;
    success = robot_cmd.get_next_value(r_speed);
    if (!success)
        return;
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    delay(500);
    analogWrite(left_f,l_speed);
    analogWrite(left_r,0);
    analogWrite(right_f,r_speed);
    analogWrite(right_r,0);
    delay(1000);
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    break;
```
Straight line driving was achieved at 190 and 248 for left and right motors respectively. 
![alt text](lab5/straightline.gif "The robot moves straight")

## Lower Limit PWM
Running the calibration script at progressively lower and lower PWM values, I found that values at 40 were the threshold for overcoming static friction in driving straight. Intuitively, the fixed 4-wheel drivetrain would experience emore friction in turns. For a point-turn, the minimum viable PWM value was 100 for each motor. 

## Open-loop Test Run
Here is the robot performing a manuever untethered and via a bluetooth command. 
![alt text](lab5/testrun.gif "The robot moves autonomously")
```C
case LAB5:
    l_speed = 190;
    r_speed = 248;
    analogWrite(left_f,l_speed);
    analogWrite(left_r,0);
    analogWrite(right_f,r_speed);
    analogWrite(right_r,0);
    delay(500);
    analogWrite(left_f,l_speed);
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    delay(500);
    analogWrite(left_f,0);
    analogWrite(left_r,l_speed);
    analogWrite(right_f,0);
    analogWrite(right_r,l_speed);
    delay(500);
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,l_speed);
    delay(500);
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    break;
```
</details>

# Lab 6
<details>

![alt text](lab6/run.gif "The robot drives and turns around")

## Data Handling
To maximize the customizability of my PID stunt code, I used many input parameters sent over bluetooth. This helped me quickly test different PID parameters as well as other parameters relevant to the stunt. 

My code takes up to 10 inputs. Some parameters operate as 'optional' parameters. 
```C
success = robot_cmd.get_next_value(kp);
            if (!success)
                return;
            success = robot_cmd.get_next_value(ki);
            if (!success)
                return;
            success = robot_cmd.get_next_value(kd);
            if (!success)
                return;
            success = robot_cmd.get_next_value(timeout);
            if (!success)
                return;
            success = robot_cmd.get_next_value(split);
            if (!success)
                split = 10;
            success = robot_cmd.get_next_value(I_gaurd);
            if (!success)
                I_gaurd = 100;
            success = robot_cmd.get_next_value(bottom);
            if (!success)
                bottom = 50;
            success = robot_cmd.get_next_value(forward);
            if (!success)
                forward = 500;
            success = robot_cmd.get_next_value(l_speed);
            if (!success)
                l_speed = 250;
            success = robot_cmd.get_next_value(r_speed);
            if (!success)
                r_speed = 250;         
```

I also used code from my IMU lab to retrieve data after recording. The most efficient way I've found to send batches of data over bluetooth is to store data to several float buffers during the run. Then, once the run is finished, send data line-by-line to a python list over bluetooth.

I can use the following callback function to process and display the data.
```python
def getData(uuid,y):
    global record
    string = y.decode("utf-8")
    if string == "DONE":
        for key in record:
            plt.plot(record[key])
            plt.title(key)
            plt.show()
        return
    datas = string.split("+")
    keys = list(record.keys())
    for data in datas:
        dat = data.split("|")
        for i,e in enumerate(dat):
            if e:
                record[keys[i]] += [float(e)]
ble.start_notify(ble.uuid['RX_STRING'], getData)
record = {"yaws":[],"output":[],"kpP":[],"kiI":[],"kdD":[]}
```

## PID 
In selecting my K values, I followed the heuristic plan laid out in lectures. I first tested a P-only system. When my P value was too high, I found that the robot would turn too fast for yaw readings to be accurate. At a certain point, especially when the robot was within 10 degrees of the set point, it struggled to overcome friction. That led me to introduce a non-zero Ki to eventually build up the magnitude of the output to overcome the friction near the setpoint. I then played with small Kd values to help improve speed of convergence. 

## Sampling
I played with the idea of truncating the yaw data to a 0-360 range. However, I found issues with the robot spinning too far past the setpoint. This is because, for a setpoint of 0, the function of P would be discontinuous. 

Therefore, I had better results with a setpoint of 180 degrees and performing no truncation on yaw. The robot does not spin far enough to encounter issues with data overflow or underflow. 

Even with my code optimized to run as fast as possible, I frequently had issues with my yaw reading accuracy. When the motors are driven too quickly, the robot easily overshoots in reality, but the sensor does not pick up on the overshoot. I suspect this has to do with a breakdown of small angle approximations as high enough angular speeds. Thus, I had to reduce my Kp and I gaurd values to reduce instances of extremely fast spinning. 

## Simple anti-windup
My implementation of anti-windup is a simple hack that truncates the integral tracking variable to a set range. 
```C
I += P * dt;
I = fmax(fmin(I,I_gaurd),-I_gaurd); // anti windup
```

## PID output to motor drivers
To convert the PID output value to integers I can send to the motor drivers, I did the following. I added a baseline speed to the output called "bottom". I then clamped to output to 0-255. I also apply a "split" to account for different efficiencies between the left and right motors. The output value can be negative, reflecting different directions of turning. I converted these real float values to positive integers and wrote them to the appropriate pins of the motor drivers. 

```C
if (output < 0) {
    analogWrite(left_f,0);
    analogWrite(left_r,(int) -fmax(output-bottom+split,-255));
    analogWrite(right_f,(int) -fmax(output-bottom-split,-255));
    analogWrite(right_r,0);
    buff0[counter] = (int) -fmax(output-bottom+split,-255);
    
} else {
    analogWrite(left_f,(int) fmin(output+bottom-split,255));
    analogWrite(left_r,0);
    analogWrite(right_f,0);
    analogWrite(right_r,(int) fmin(output+bottom+split,255));
    buff0[counter] = (int) fmin(output+bottom-split,255);
```

(ADD GRAPHS)

</details>