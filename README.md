# Max-Desparate-New-Fast-Robots-Site.github.io
I give up on a fancy website (too many hours lost over broken js files) and desparately try to get a new one going. 

# Lab 4

<details markdown="1">
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

<details markdown="1">

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

<details markdown="1">

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

Driving in the hallway, the robot begins to turn and converge toward 180 degrees.

![alt text](lab6/yawinmotion.png "The robot drives and turns around after driving forward")

The PWM values that the motors experience look like this.

![alt text](lab6/outputsinmotion.png "PWM values ")

This output can be broken down into the trends of Kp\*P, Ki\*I, and Kd\*D.
![alt text](lab6/P.png "P component")

![alt text](lab6/I.png "I component")

![alt text](lab6/D.png "D component")

</details>

# Lab 7

<details markdown="1">

## Data capture
In the Upson lab, I ran a modified version of the TOF lab code to get step response data. I drove the motors at a set speed for 1 second before letting the robot coast and letting drag slow the robot to a stop.

For easier conversion of u to pwm values, I ran the step response at the maximum PWM value that would allow for straight driving.

Most critically, it was important to set the robot far enough for the robot to come to a rolling stop without collision.

![alt text](lab7/xdata.png "Raw distance measurements")

I processed the data to derive velocity and acceleration measurements. I used symmetric moving averages just to smooth out the data. Without the smoothing, errors from the distance measurements propogate through the differentiation. With a little bit of smoothing, I am much more confident in reading the graphs for rise time.

```python
import numpy as np
def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'same') / w

xdot = moving_average(np.ediff1d(x)/np.ediff1d(times),3)
```

![alt text](lab7/xdotdata.png "Smoothed velocity")

Here we can see the robot move towards the wall with nearly continuous acceleration and increasing speed. After 1 second from the start of data capture, acceleration reverses direction and a lower magnitude acceleration at a point in time that matches the inflection point in the X data.

## Drag and Mass
After this, the process of capturing the 90% rise time is relatively simple implementation of the lecture code. 
```python
# d ~ 1/xdot_steady
xdot_steady = np.min(xdot)
d = (-1/xdot_steady)

xdot90 = 0.9*xdot_steady
t_rise = (times[np.argmax(xdot < xdot90)] - times[0]) 

# m = -d*t_0.9 / ln(0.1)
m = -d*t_rise / np.log(0.1)

print(d, m)
>> 0.6709160379958043 264.7416977038248
```

## A and B matrices
With my coefficients determined, I then calculate my A, B, and C matrices. I also discretize A and B as required. 
```python
# xdot = Ax + Bu
A = np.array([[0,1],[0,-d/m]])
B = np.array([[0],[1/m]])

C = np.array([[-1,0]])

#discretized, dt is sampling time
dt = (times[-1] - times[0]) / len(times)
Ad = np.eye(len(A)) + dt * A  
Bd = dt * B

A = Ad
B = Bd
```


|  A  |         |
|----|---------|
| 1 | 97.32545098 |
| 0 | 0.75335504 |

|  B  |
|----|
| 0. | 
| 0.36762419 |

|  C  |         |
|----|---------|
| -1 | 0 |

## Applying Kalman Filter
Then, it is a matter of applying the kalman filter with the kalman filter function on the measured data.

We use this function applied to a given motor input u and measured output state y.
```python
sigma_1, sigma_2, sigma_3 = dt, dt, 27.

sig_u=np.array([[sigma_1**2,0],[0,sigma_2**2]]) 
sig_z=np.array([[sigma_3**2]])

sigma = np.array([[2500,0],[0,10]])


def kf(mu,sigma, u, y):
    mu_p = A.dot(mu) + B.dot(u)
    sigma_p = A.dot(sigma.dot(A.transpose()))+sig_u
    
    sigma_m = C.dot(sigma_p.dot(C.transpose()))+sig_z
    kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))
    
    y_m = y-C.dot(mu_p)
    mu = mu_p+kkf_gain.dot(y_m)
    sigma=(np.eye(2)-kkf_gain.dot(C)).dot(sigma_p)

    return mu, sigma
```

I first tested the following sigmas. My position and speed standard deviation is about 9.9. I try 20 for my sigma_3 process noise. 

```python
sigma_1, sigma_2, sigma_3 = np.sqrt(dt), np.sqrt(dt), 20.


sig_u=np.array([[sigma_1**2,0],[0,sigma_2**2]]) 
sig_z=np.array([[sigma_3**2]])

sigma = np.array([[2500,0],[0,10]])
```

I used the below code to visualize the filtered data
```python
kf_state = []
x_ = -np.array([x[1],0]).transpose()
Y = -np.stack([x[1:],xdot[0:]]).transpose()
u = 1
for i,y in enumerate(Y):
    if i == np.argmin(x)-1:
        u = 0
    x_, sig = kf(x_, sigma, [[u]], y)
    kf_state.append(x_)

kf_state = np.stack(kf_state)
plt.plot(kf_state[:,0,1],label="Kalman Filtered")
plt.plot(xdot[1:], label="Raw")
plt.legend()
plt.show()
```

The results show the filtered data means consistently overshooting measured results, resulting in the robot appearing to be closer to the wall than it is in the measured data. If anything, with the doppler effect, appearing slightly further from the wall would be more reasonable.
![alt text](lab7/kalman0.png "First Kalman test")

To induce a somewhat tighter fit, I increase sigmas in my process values. 
I use 97.3, 9.9, and 50 for my sigmas 1 through 3. And I use 2.5 and 243 for m
![alt text](lab7/kalman1.png "Second Kalman test")

## Onboard extrapolation

To interpolate between TOF readings on the Artemis, I use my last readings of x and xdot to linearly extrapolate into the future. This allows me to have a finer sampling rate. 

After capturing the initial state with the TOF sensor, I am able to begin applying the extrapolation to incoming data. When real sensor data is not available, the Artemis falls back on, every 15 milliseconds, at a much faster sampling rate, a best guess of position and speed to predict the present state. 

```C
while (true) {
    if (distanceSensor1.checkForDataReady()) {
        last_time = millis();
        distance1 = distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor1.startRanging();
        x_(0,0) = distance1;
        break;
    }
    delay(5);
}

analogWrite(right_f,r_speed);
analogWrite(right_r,0);
analogWrite(left_f,l_speed);
analogWrite(left_r,0);

while (counter < buffer_size) {
    timestamp = micros();
    if (timestamp - last_time2 > timeout) {
        analogWrite(right_f,0);
        analogWrite(right_r,0);
        analogWrite(left_f,0);
        analogWrite(left_r,0);
        u(0) = 0;
        if (timestamp - last_time2 > 5000000) {
            break;
        }
    }

    if (distanceSensor1.checkForDataReady()) {
        distance1 = distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor1.startRanging();

        if (counter > 0) {
            dt = millis() - last_time;
            last_time += dt;
            xdot = (distance1 - buff0[last_counter]) / dt;
            last_counter = counter;
        }
        
        y = {distance1,xdot};

        times[counter] = timestamp/1000.;
        distance1 = y(0,0);
        buff0[counter] = distance1;
        buff1[counter] = y(0,1);
        counter++;
        }

    delay(15); // Fallback interpolation
    times[counter] = timestamp/1000.;
    distance1 += 15*y(0,1);
    buff0[counter] = distance1;
    buff1[counter] = y(0,1);
    counter++;

}
```

Here are the results of the interpolation in a sample run. The x axis shows the number of samples. The linear interpolation strategy works very well during most of the motion. However, where the magnitude of velocity is changing much faster, near the start and end of the robot's motion, the constant speed assumption breaks down. 
![alt text](lab7/interpx.png "X data with extrapolation")

</details>

# Lab 8

<details markdown="1">

## Overview
I chose to perform Task B: Orientation Control. For this task, I will program the robot to drive fast toward a wall, drift and turn 180 degrees and drive back. 

## Best run
![alt text](lab8/bestrun.gif "Best run")
In my best run, my robot completes the task in about 4 seconds. 

## Blooper reel
[![alt text](https://img.youtube.com/vi/L2kpKNiJ4Ls/0.jpg)](https://www.youtube.com/watch?v=L2kpKNiJ4Ls)

## Implementation
For this high speed task, I must face a few specific challenges. I am aware that the robot's yaw measurement is inaccurate at fast rotations. It may be necessary to compensate for expected drift by programming the turn setpoint as required. 

My code consists of three loops that the robot progresses through. First, the robot drives forward until it detects a forward distance less than 914 mm (3 feet). Then it initates a 180 degree turn using PID. Finally, it drives straight back to the finish line. At every stage, there exists a timeout in the case that any phase hangs for any technical reason. This helps prevent the robot from behaving too erratically and getting damaged.

For faster distance updates, I use linear extrapolation to estimate distance when the ToF data is unavailable. Below is the code for phase 1.  I found that setting *distance_to_wall* to values higher than 914 mm gives the robot enough distance to respond, giving enough space for the arc of the turn. Otherwise the robot ends up crashing into the wall. 
```python
while (distance1 > distance_to_wall) {
    timestamp = micros();
    if (timestamp - last_time2 > timeout || counter >= buffer_size) {
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    if (timestamp - last_time2 > 5000000) {
        break;
    }
    }

    if (distanceSensor1.checkForDataReady()) {
    distance1 = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    distanceSensor1.startRanging();

    if (counter > 0) {
        dt = millis() - last_time;
        last_time += dt;
        xdot = (distance1 - buff0[last_counter]) / dt;
        last_counter = counter;
    }
    
    times[counter] = timestamp/1000.;
    buff0[counter] = distance1;
    buff1[counter] = xdot;
    counter++;
    }

    delay(15);
    times[counter] = timestamp/1000.;
    distance1 += 15*xdot;
    //buff0[counter] = distance1;
    //buff1[counter] = xdot;
    //counter++;

}
```

And here is phase 2. This code is largely unchanged from the PID lab. Having a setpoint other than 180 gave me flexibility in adjusting for yaw overshoot or undershoot. 
```python
last_time = micros();

while (counter < buffer_size) {
    // if data ready and past minimum update time
    Serial.println(counter);
    if(myICM.dataReady())
    {
    // collect data
    myICM.getAGMT();
    timestamp = micros();
    dt = (timestamp - last_time2)/1000000.;
    last_time2 = timestamp;
    yaw_g += myICM.gyrZ()*dt;

    yaws[counter] = yaw_g;

    // calculate P error
    P = set_point - yaw_g;

    // calculate I error
    I += P * dt;
    I = fmax(fmin(I,I_gaurd),-I_gaurd); // anti windup

    // calculate D error
    D = (P - P_prev) / dt;
    P_prev = P;

    // calculate output, 
    output = kp * P + ki * I + kd * D;
    //buff0[counter] = output;
    buff1[counter] = kp*P;
    buff2[counter] = ki*I;
    buff3[counter] = kd*D;

    counter++;

    // clamp output 
    if (output < 0) {
        
        analogWrite(right_f,0);
        analogWrite(right_r,(int) -fmax(output-bottom+split,-255));
        analogWrite(left_f,(int) -fmax(output-bottom-split,-255));
        analogWrite(left_r,0);
        //buff0[counter] = (int) -fmax(output-bottom+split,-255);
        //buff2[counter] = l_speed;
        
    } else {
        analogWrite(right_f,(int) fmin(output+bottom-split,255));
        analogWrite(right_r,0);
        analogWrite(left_f,0);
        analogWrite(left_r,(int) fmin(output+bottom+split,255));
        //buff0[counter] = (int) fmin(output+bottom-split,255);
        //buff1[counter] = r_speed;
        //buff2[counter] = l_speed;
    }

}
```

And finally, I have a final drive forward script for phase 3. This part is the most straightforward.
```python
 if (timestamp - last_time > timeout) {
    delay(5);
    analogWrite(right_f,r_speed);
    analogWrite(right_r,0);
    analogWrite(left_f,l_speed);
    analogWrite(left_r,0);
    delay(forward);
    analogWrite(right_f,0);
    analogWrite(right_r,0);
    analogWrite(left_f,0);
    analogWrite(left_r,0);
    break;
    }
```

## PID and task Settings
* Kp = 0.2
* Ki = 0.007
* Kd = 0.02
* Timeout setting = 0.8 seconds
* Setpoint = 90 degrees

## Plots
I plot data captured from my best run. When the data shows zero, that indicates that the data isn't captured for phase 1. 

![alt text](lab8/yaws.png "Yaws plot")

![alt text](lab8/distances.png "Distances plot")

![alt text](lab8/speeds.png "Speeds plot")

![alt text](lab8/kpp.png "KpP plot")

![alt text](lab8/kii.png "KiI plot")

![alt text](lab8/kdd.png "KdD plot")

</details>

# Lab 9

<details markdown="1">

For the mapping task, I chose to repurpose my PID orientation control code to perform orientation control. I perform PID on position setpoints around a 360 degree turn. Specifically, in time windows of about a second, a running record of the set point position gets iterated up by 24 degrees, or a 15th of a full turn. The robot performs PID to try to reach that set point. After the time window is over, a measurement is taken of the distance from both time of flight sensors (mounted perpendicular to eachother) and of the current yaw reading. The yaw reading is known to be pretty inaccurate for rotations that are too fast, so I keep the kP term relatively small. 

The functional Arduino code looks something like this.

```C
yaws[counter] = yaw_g;
analogWrite(right_f,255);
analogWrite(right_r,255);
analogWrite(left_f,255);
analogWrite(left_r,255);
delay(50);

target += 360/15;

distanceSensor0.startRanging();
while (true) {
if (distanceSensor0.checkForDataReady()) {
distance0 = distanceSensor0.getDistance();
distanceSensor0.clearInterrupt();
distanceSensor0.stopRanging();
d0s[counter] = distance0;
break;
}
delay(5);
}

distanceSensor1.startRanging();
while (true) {
if (distanceSensor1.checkForDataReady()) {
distance1 = distanceSensor1.getDistance();
distanceSensor1.clearInterrupt();
distanceSensor1.stopRanging();
d1s[counter] = distance1;
break;
}
delay(5);
}

counter++;
```

Once I resolve the bluetooth connection issues, the code runs smoothly and captures and sends data well over bluetooth. The code for the data transfer is recycled from other labs. 

![alt text](lab9/spin.gif "Example of robot spinning and mapping")

## Robot Spinning
I struggled to get the robot to turn in place, even while using duct-taped wheels. In addition, the ToF sensors are also offset from the center of the robot. Finally, there is likely a discrepency between the yaw the robot measures and the actual yaw.

I try to later account for these offsets in my plotting of my datapoints. This is explained in the next sections.

## Plots
When I recieve raw data from the data callback, I recieve data for yaw, distance0, and distance1. 
![alt text](lab9/yaws.png "Raw yaw data")
![alt text](lab9/d0s.png "Distance 0 data")

I then convert this data to polar plots after yaws are converted to radians.
![alt text](lab9/polar.png "Polar plot of ToF")

## Precision and repeatability
Running the same code twice at a given location produces data that is surprisingly similar. This data is without any additional processing. This result occurs inspite of the fact that the starting position of the robot is not carefully controlled and the fact that the robot does not turn on its own axis. 

![alt text](lab9/polarprecision.png "Polar plot of ToF")

## Combining maps
From yaw and distance measurement data, we can convert polar coordinates to planar coordinates. I used the expression *x = r\*cos(theta) + x_o* and *y = r\*sin(theta) + y_o* to transform the data to a form that can be superimposed onto eachother. *x_o* and *y_o* refer to the respective origins at which the data was collected in the map. 

In python, the resultant code looks like this:

```python
import numpy as np
import matplotlib.pyplot as plt

keys = [["neg3neg2",-3,-2],["03",0,3],["53",5,3],["5neg3",5,-3]]

for name, delx, dely in keys:
    r = np.load(name+"r1.npy")+75
    theta = np.load(name+"theta.npy")
    x = (r)*np.cos(theta/180*np.pi+1*np.pi/2)/304.8+delx
    y = (r)*np.sin(theta/180*np.pi+1*np.pi/2)/304.8+dely
    plt.plot(x,y, ".",)
    plt.plot([delx], [dely], "x", label =  name)
plt.legend()
plt.show()
```
Thetas are offset by 90 degrees to account for my starting position. And distances are scaled to imperial units. 

![alt text](lab9/raw_map.png "Raw unprocessed map")

From the unprocessed map of the raw data, we can see an overall outline of the room and dots in the vague positions of the boxes within the room. To get more out of my data, I account for the fact that my ToF sensor travels a radius away from my center of rotation. Additionally, when my robot thinks it has completed a 360 degree turn, more often the turn is closer to 340. We can scale theta to try to correct this, too.

Adjusting the data to better fit, accounting for different offsets, and radii of rotation, and incomplete turns, I am able to produce a much cleaner joint point cloud.

```python
keys = [["neg3neg2",-3,-2, 'red'],["03",0,3, 'green'],["53",5,3,'blue'],["5neg3",5,-3,'purple']]
for name, delx, dely, color in keys:
    r = np.load(name+"r1.npy")
    theta = np.load(name+"theta.npy")
    if name == "03":
        x = (r+25)*np.cos(theta*(165/180)/180*np.pi+np.pi/2+0.4)/304.8+delx
        y = (r+25)*np.sin(theta*(165/180)/180*np.pi+np.pi/2+0.4)/304.8+dely+0.3
    elif name == "53":
        x = (r+65)*np.cos(theta*(200/180)/180*np.pi+np.pi/2-0.6)/304.8+delx
        y = (r+65)*np.sin(theta*(200/180)/180*np.pi+np.pi/2-0.6)/304.8+dely
    elif name == '5neg3':
        x = (r+75)*np.cos(theta*(170/180)/180*np.pi+np.pi/2-0.)/304.8+delx
        y = (r+75)*np.sin(theta*(170/180)/180*np.pi+np.pi/2-0.)/304.8+dely
    else:
        x = (r+115)*np.cos(theta*(170/180)/180*np.pi+np.pi/2-0.1)/304.8+delx
        y = (r+115)*np.sin(theta*(170/180)/180*np.pi+np.pi/2-0.1)/304.8+dely
    plt.plot(x,y, ".", color=color)
    plt.plot([delx], [dely], "x", color=color, label =  name)
plt.legend()
plt.show()
```

![alt text](lab9/cleaned_map.png "The cleaned map")

I can draw the predicted room boundaries using horizontal and vertical lines. I construct these lines under the basic assumption that all boundaries are strictly horizontal and vertical. For a vertical line, for example, I take a set of points, and draw a vertical line at *x = x_mu* where *x_mu* is the sample mean x position.

```python
p = np.stack(points, 1).reshape((2,-1))
plt.plot(p[0],p[1],".")

s = p[:,p[0,:]>6.1]
f = np.array([[min(s[1]),np.mean(s[0])],[max(s[1]),np.mean(s[0])]]).transpose()
#plt.plot(s[0], s[1],".")
plt.plot(f[1], f[0])

s = p[:,p[1,:]>3.5]
f = np.array([[np.mean(s[1]),np.min(s[0])],[np.mean(s[1]),np.max(s[0])]]).transpose()
#plt.plot(s[0], s[1],".")
plt.plot(f[1], f[0])

s = p[:,p[0,:]>-3]
s = s[:,s[0,:]<-2]
s = s[:,s[1,:] > 0.]
f = np.array([[min(s[1]),np.mean(s[0])],[max(s[1]),np.mean(s[0])]]).transpose()
#plt.plot(s[0], s[1],".")
plt.plot(f[1], f[0])

#...
```

![alt text](lab9/line_overlay.png "The cleaned map")

## Prepping for simulator
From these points, it is a simple to connect adjacent points to build a numpy array of vertices throughout the environment for later use in the simulator.


![alt text](lab9/single_line.png "The cleaned map")

</details>

# Lab 10 

<details markdown="1">

Code is adapted and optimized from Aarya Pai's 2022 lab work.

Here is the complete run with Bayes filter applied on a trajectory through the simulated environment.


![alt text](lab10/run.gif "Simulator run")

## Inferences and results
The Bel index peaks at a probability of aroud 0.5, but most of the time resides below 0.2. Error is smallest around when the robot moves relatively consistently and at short distances. Large movements especially with a lot of rotation reintroduce the most error. Interestingly, the GT alpha appears to grow substantially over the course of the trajectory. This may be an error with the simulator, given that we expect angles to be constrained -180 to 180. This is room for further investigation in future labs. 


![alt text](lab10/prediction_stats.png "Comparing predictions and ground truth")


## compute_control
To clean up my code for readability, I create delta_x and delta_y which is passed to the numpy arctan2 function. I use numpy functions where possible to leverage the package's computational speed advantage. I also take advantage of the array and matrix operations in calculating delta_trans. 
```python
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    delta_x = cur_pose[0] - prev_pose[0]
    delta_y = cur_pose[1] - prev_pose[1]
    delta_rot_1 = mapper.normalize_angle(np.rad2deg(np.arctan2(delta_y, delta_x) - prev_pose[2]))
    delta_trans = np.sum((np.array(cur_pose[:2]) - np.array(prev_pose[:2 ]))**2)**0.5
    delta_rot_2 = mapper.normalize_angle(cur_pose[2] - prev_pose[2] - delta_rot_1)

    return delta_rot_1, delta_trans, delta_rot_2
```

## odom_motion_model
np.product can be used to take the product across all entries in an array. I used this fact to reduce unnecessary indexing. 

```python
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)


    Returns:
        prob [float]: Probability p(x'|x, u)
    """  
    u_hat = compute_control(cur_pose, prev_pose)
    sigma = np.array((loc.odom_rot_sigma, loc.odom_trans_sigma, loc.odom_rot_sigma))
    prob = np.product(loc.gaussian(u_hat, np.array(u), sigma))
    
    return prob
```

## prediction_step

I experimented with matrix operations and other numpy tools to reduce the runtime of this function further. I was not successful in finding an alternative outside of the 6 for-loops that worked for my case. The main challenge is the need to apply the belief update between every combination of previous and current poses. With larger environments, this significantly reduces the performance of the Bayes filter, given that the sum connections follows the pyramid numbers formula *n(n+1)/2* or approximately O(n**2).

Here, I also experimented with adding a small epsilon to the division to prevent division errors. In practice, I did not run into issues with the normalization.

```python
def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    
    actual_u = compute_control(cur_odom, prev_odom)
    loc.bel_bar = np.zeros((mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A))
    for x_prev in range(mapper.MAX_CELLS_X):
        for y_prev in range(mapper.MAX_CELLS_Y):
            for a_prev in range(mapper.MAX_CELLS_A):
                if loc.bel[x_prev,y_prev,a_prev] > 0.0001:
                    for x_t in range(mapper.MAX_CELLS_X):
                        for y_t in range(mapper.MAX_CELLS_Y):
                            for a_t in range(mapper.MAX_CELLS_A):
                                loc.bel_bar[x_t,y_t,a_t] += loc.bel[x_prev,y_prev,a_prev] * odom_motion_model(mapper.from_map(x_t,y_t,a_t),mapper.from_map(x_prev,y_prev,a_prev),actual_u)

    loc.bel_bar /= np.sum(loc.bel_bar)#+0.00001   
```

## sensor_model and update_step
These functions are pretty lean as-is. I could not make improvements here.
```python
def sensor_model(obs):
    """ This is the equivalent of p(z|x).

    Args:
        obs ([ndarray]): A 2D array consisting of the true observations for each robot pose in the map 

    Returns:
        [ndarray]: Returns a 1D array of size N (total number of cells in the map) with the likelihoods of each cell's observations
    """
    return np.array([loc.gaussian(loc.obs_range_data[i], obs[i], loc.sensor_sigma) for i in range(mapper.OBS_PER_CELL)])



def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    for x_t in range(mapper.MAX_CELLS_X):
        for y_t in range(mapper.MAX_CELLS_Y):
            for a_t in range(mapper.MAX_CELLS_A):
                loc.bel[x_t,y_t,a_t] = loc.bel_bar[x_t, y_t, a_t] * np.prod(sensor_model(mapper.obs_views[x_t,y_t,a_t]))
                
    loc.bel /= np.sum(loc.bel)
```

</details>

# Lab 11

<details markdown="1">

## Localization in Simulation
I tested the simulation's localization. The results appeared virtually identical to the previous lab. We can observe the belief trajectory in blue fitting fairly tightly to the ground truth in green. By testing in simulation first, we can isolate errors between code and real-world physics. From the simulated results, we have a benchmark for the error in an ideal case. 

![alt text](lab11/finalplot.png "Final plot of odom, ground truth, and belief")

## Real localization
Next, we test the same code on the real robot. I implemented the *perform_observation_loop* code as follows. 

```python
async def perform_observation_loop(self, rot_vel=120):
        """Perform the observation loop behavior on the real robot, where the robot does  
        a 360 degree turn in place while collecting equidistant (in the angular space) sensor
        readings, with the first sensor reading taken at the robot's current heading. 
        The number of sensor readings depends on "observations_count"(=18) defined in world.yaml.
        
        Keyword arguments:
            rot_vel -- (Optional) Angular Velocity for loop (degrees/second)
                        Do not remove this parameter from the function definition, even if you don't use it.
        Returns:
            sensor_ranges   -- A column numpy array of the range values (meters)
            sensor_bearings -- A column numpy array of the bearings at which the sensor readings were taken (degrees)
                               The bearing values are not used in the Localization module, so you may return a empty numpy array
        """
        # Get ArtemisBLEController object
        global flag
        global record
        flag = False
        
        # Run Lab 9 code for mapping
        ble.send_command(CMD.LAB9,".07|0.005|0.02|5000000|25|40|120|1000|255|240")

        # Awaiting flag to be turned on (when data is done sending)
        while not flag:
            await sleep()
        
        # Light post-processing akin to Lab 9 for angle and range correction
        sensor_ranges = np.array(record["D1"])
        sensor_ranges = (sensor_ranges + 75)/1000
        sensor_ranges = np.expand_dims(sensor_ranges,1)
        sensor_bearings = np.array(record["yaws"])
        sensor_bearings = sensor_bearings*(170/180)/180*np.pi
        sensor_bearings = np.expand_dims(sensor_bearings,1)
        
        
        return sensor_ranges, sensor_bearings
        
flag = False
```

Running this code at one of the four marked poses in the environment (5 ft, -3ft, 0 deg) looks something like this. 

![alt text](lab11/singlespin.gif "Single example fo the robot spinning on a coordinate")

Even with this imperfect data capture trajectory, we get a predicted position very close to the actual point. 

![alt text](lab11/prediction.png "Single example fo the robot spinning on a coordinate")

The actual position is (-1.524m, -0.914m, 0 degrees). In other words, the belief from the update step is nearly exactly the actual position.

## The Other Poses

Due to the large detection radius, the size of the prediction error ranges up to 1 foot. 

### Ground truth (5, 3), belief (5,2)
![alt text](lab11/53.png "Prediction at 5,3")

### Ground truth (0, 3), belief (1,3)
![alt text](lab11/03.png "Prediction at 0,3")

### Ground truth (-3, -2), belief (-3,-2)
![alt text](lab11/neg3neg2.png "Prediction at -3,-2")

</details>

# Lab 12

<details markdown="1">

## Overview
I had many challenges in tackling this lab. In this lab, the task is to autonomously navigate through a long sequence of waypoints in a predefined map. 

```
1. (-4, -3)    <--start
2. (-2, -1)
3. (1, -1)
4. (2, -3)
5. (5, -3)
6. (5, -2)
7. (5, 3)
8. (0, 3)
9. (0, 0)      <--end
```

In order to achieve this, I considered several strategies. Below, I explain the sequence of strategies I tested.


| # | Waypoints | Distance | Yaw |
| - | --- | --- | ---|
|1 | Bayes filter update every step | TOF with interpolation | PID yaw position control | |
|2 | Bayes filter at selected points | TOF with interpolation | PID yaw position control | |
|3 | Dead reckoning | TOF with PID position control | PID yaw position control 
| 4 | Mixed dead reckoning and Bayes filter | TOF with PID position control (one-shot style) | PID yaw position control

## Method 1

[![alt text](https://img.youtube.com/vi/hNxk1YXVzCo/0.jpg)](https://www.youtube.com/watch?v=hNxk1YXVzCo)

I reused lab 7 code for TOF with interpolation. I adapted my PID lab code for the yaw control. These two steps were the more straightforward. I removed the data capture part of these functions and rebundled them as the DRIVE and TURN case. They would allow me to command the robot to drive x distance or turn y degrees, with other parameters to control the motion.

The DRIVE function works by:

1. Recording the initial distance ahead with TOF 
2. Determining the distance to stop at by subtracting the initial distance by the desired movement distance.
3. Move forward until the step 2 distance is reached. 

For the Bayes filter update, I used asyncio to allow for the python kernel to wait until the bearings and ranges are populated before proceeding. 

```python
async def perform_observation_loop(self, rot_vel=120):

    global flag
    global record
    global r_offset
    flag = False
    
    print("send")
    record = {"yaws":[],"D1":[]}
    ble.send_command(CMD.LAB9,"2.|0.7|0.8|5000000|-30|100|150|1000|255|255")
    
    last = time()
    while time() < last + 40:
        if len(record["D1"]) == 15:
            print("type 2 break")
            break
        await sleep()
    if time() > last + 40:
        print("timed out")
    
    sensor_ranges = np.array(record["D1"])
    sensor_ranges = (sensor_ranges + r_offset)/1000
    sensor_ranges = np.expand_dims(sensor_ranges,1)
    sensor_bearings = np.array(record["yaws"])
    sensor_bearings = (sensor_bearings + 20)*(170/180)/180*np.pi
    sensor_bearings = np.expand_dims(sensor_bearings,1)
    
    return sensor_ranges, sensor_bearings
```

The script includes preliminary processing of the raw data to improve localization accuracy.

To navigate between waypoints, these wrapper and helper functions are used. 

```python
waypoints = [(-4, -3),   
            (-2, -1),
             (1, -1),
             (2, -3),
             (5, -3),
             (5, -2),
             (5, 3),
             (0, 3),
             (0, 0)]

def getAngle(arr):
    angle = np.rad2deg(np.arctan2(arr[1],arr[0]))
    if angle > 180:
        angle = -360 + angle
    return angle
    
def getDistance(arr):
    return np.linalg.norm(arr)*1000

async def turn(d_angle):
    global flag
    
    angle_left = d_angle
    large_angle_threshold = 90

    if abs(angle_left) > large_angle_threshold:
        angle_left = angle_left - np.sign(angle_left) * large_angle_threshold
        flag = False
        #                           P   I    D   timeout s Igaurd bottom target
        ble.send_command(CMD.TURN,"2.|0.9|0.8|2000000|100|220|160|"+str(np.sign(angle_left)*large_angle_threshold))
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()
        
        await turn(angle_left)
    else:
        flag = False
        ble.send_command(CMD.TURN,"2.|0.9|0.8|2000000|100|220|160|"+str(angle_left))
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()   
```

The turning function seperates large turns into a sequence of smaller turns to reduce errors from yaw drift. These pieces of code make it easier for me to iterate between waypoints, determining the next desire, and getting the angle and distance needed to travel. 

```python
 waypoint_m = np.array(waypoints[i+1])/3.2808399
desire = np.subtract(waypoint_m,current_belief[:-1])

print("Desire", desire)

d_angle = getAngle(desire)
d_dist = getDistance(desire)

print(f"Desired angle {d_angle}, desired distance {d_dist}")
last = time()
while time() < last + 1:
    await sleep()
await turn(d_angle - curr_angle) 
curr_angle = d_angle

print("done turn with current ", curr_angle)

flag = False
print("send drive")
ble.send_command(CMD.DRIVE,"100|115|"+str(int(d_dist))+"|5000000|0.0015|0.016|0.05|200")

last = time()
while not flag and time() < last + 10:
    await sleep()
```

## Method 2
My localization step captures 15 data points but does not complete a full turn. This led to me having issues with compounding issues with yaw drift. Furthermore, the Bayes filter update step error for yaw was much greater than the error for position. That is why most of my code omits use of the localized yaw angle.

To increase ease of testing and reduce yaw drift, I decided to selectively choose to do localization at some waypoints. Particularly around the 3rd or 4th waypoint, my position error was greatest. Ultimately I had issues with this approach too since the TOF sensor would bug out frequently going between the localization and driving commands. It was incredibly strenous to debug this. More about this in the 'troubleshooting' section.

I only successfully reached the fourth or fifth waypoint with this approach. 

## Method 3
[![alt text](https://img.youtube.com/vi/GTal47eFbI8/0.jpg)](https://www.youtube.com/watch?v=GTal47eFbI8)

I observed that some groups seemed to get much more consistent results with a PID loop on translation control. The challenge of using TOF sensing for distance control is that, when yaw drifts too far or when the robot does not drive perfectly straight, it is common for other objects to come in and out of view, changing the robot's belief of its distance. 

One approach to fix this may be to apply a (Kalman) filter on the distance readings. Alternatively, I noticed that choosing to occasionally drive backwards could mean TOF sensing occurs on a more consistent surface.

I determine the distance and angle to move by using the euclidian distance between the present and next waypoints. I am able to tune the angle and distance to move at every step as needed.


## Troubleshooting
I faced notable issues with my TOF sensor. There were several instances where the sensor would abruptly stop recording distances. Without clear documentation on what the Sparkfun library functions do, any kind of debugging was done with intuition. 

I began to notice that other teams were implementing, testing, and finishing the lab in much faster time with purely open-loop control. This led me to try simpler and simpler approaches. However, my robot is particularly finnicky with it's sensors, which I think inhibited my progress.

I believe the following codeblock is what was causing me the most issues. In this code, I try to turn on the TOF sensor, wait until I have data, then set the distance value to the measurement. There is also a timeout script so that my robot does not get trapped in an infinite loop (which occurs frequently before this implementation). This code appears in my translation PID script. Frequently, on the second or third run of the same command, data would fail to ever 'get ready.'

```C
distanceSensor1.startRanging();
while (true) {
    if (distanceSensor1.checkForDataReady()) {
    distance1 = distanceSensor1.getDistance();
    Serial.print("Inside ");
    Serial.println(distance1);
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    break;
    } else if (micros() - timestamp > 25000 || micros() - last_time2 > timeout) {
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    break;
    }
    delay(5);
}
```

This code is largely the same for the Sparkfun library's one-shot measurement example. But for, repeated loops of getting data, it is incredibly unreliable. 

Digging more into the codebase, I found that I might have better results by starting ranging at the very beginning of my code and using clearInterrupt exclusively to reset the sensor measurement. Alternatively, I might try using startOneshotRanging instead of startRanging. 

For the one-shot ranging technique, I used the following code. 
```C
distanceSensor1.startOneshotRanging(); 
while (!distanceSensor1.checkForDataReady())
{
delay(1);
}
distance1 = distanceSensor1.getDistance(); 
```

## UPDATE
After I implemented the oneshot ranging code for all instances where I measure distance, my TOF sensor worked reliably and without issue. 


## Method 4
I noticed my yaw bearings progressively get worse with each waypoint. 

I found benefits to using different PID parameters for position control depending on the desired distance to travel. The parameters that were sensible for short ranges did not apply well for longer distances. It was easier to create different PID commands for different distances than to trial-and-error a one-size-fits-all solution for all distances.

To simplify the driving code, I wrote the following function.

```python
async def drive(d_dist):
    global flag
    flag = False
    d_dist = int(d_dist)
    if abs(d_dist) < 400:
        ble.send_command(CMD.DRIVE,"100|117|"+str(d_dist)+"|5000000|0.0018|0.08|0.05|5000")
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()
    elif abs(d_dist) < 600:
        ble.send_command(CMD.DRIVE,"100|117|"+str(d_dist)+"|5000000|0.0015|0.016|0.05|200")
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()
    elif abs(d_dist) < 1300:
        ble.send_command(CMD.DRIVE,"100|117|"+str(d_dist)+"|5000000|0.0015|0.01|0.05|2000")
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()
    else:
        ble.send_command(CMD.DRIVE,"100|105|"+str(d_dist)+"|5000000|0.0007|0.025|0.005|4000")
        last = time()
        while time() < last + 10:
            if flag:
                break
            await sleep()
```

I reasoned that after the largest range movements is when localization is most valuable to correct errors in positioning. So I added back a localization at (5,3).

After these changes, I was able to complete the entire run. I ran out of time for more tuning, but with more tuning, accuracy would be even better. 

[![alt text](https://img.youtube.com/vi/D-quUX-ZyAc/0.jpg)](https://www.youtube.com/watch?v=D-quUX-ZyAc)

The following code was the main loop used to perform my best run. You can see the combination of PID closed loop control and Bayes filter localization.

```python

r_offset = 35

loc.init_grid_beliefs()

hist_x = []
hist_y = []

curr_angle = 0

waypoints = [(-4, -3), 
            (-2, -1),
             (1, -1),
             (2, -3),
             (5, -3),
             (5, -2),
             (5, 3),
             (0, 3),
             (0, 0)]


for i, waypoint in enumerate(waypoints):
    global flag
    flag = False

    if waypoint == (-4,-3):
        await drive(570)
        await turn(75)
        await drive(-600)
        curr_angle = -180
    elif waypoint == (-2, -1):
        await drive(-900)
    elif waypoint == (1,-1):
        await turn(110)
        await drive(500)
        await turn(46)
        curr_angle = 0
    elif waypoint == (2, -3):
        await drive(700)
        await turn(80)
    elif waypoint == (5,-3):
        await drive(300)
    elif waypoint == (5,-2):
        await drive(1524)
        curr_angle = 90

    current_belief = np.divide(waypoint,3.2808399)
    current_belief = np.append(current_belief,curr_angle)

    if waypoint == (5,3):

        ## Localization step
        record = {"yaws":[],"D1":[]}
        await loc.get_observation_data()
        loc.update_step()
        loc.plot_update_step_data(plot_data=True)
        argmax_bel = get_max(loc.bel)
        current_belief = loc.mapper.from_map(*argmax_bel[0])
        curr_angle = current_belief[-1]

        ## Get desire (distance and angle to next waypoint)
        waypoint_m = np.array(waypoints[i+1])/3.2808399
        desire = np.subtract(waypoint_m,current_belief[:-1])
        d_angle = getAngle(desire)
        d_dist = getDistance(desire)

        ## Execute desires
        if abs(d_angle + 360 - current_belief[-1]) < abs(d_angle - current_belief[-1]):
            await turn(d_angle + 360 - curr_angle)
        else:
            await turn(d_angle - curr_angle)
        curr_angle = d_angle
        await drive(d_dist)

    elif waypoint == (0,3):
        if abs(180 - curr_angle) < abs(-180 - curr_angle):
            await turn(180 - curr_angle)
        else:
            await turn(-180 - curr_angle)
        await turn(-80)
        await drive(-900)

    hist_x.append(current_belief[0])
    hist_y.append(current_belief[1])
    plt.plot(hist_x,hist_y,"-x")
    plt.xlim(-5,5)
    plt.ylim(-5,5)
    plt.show()
```

The following plot shows the robot's belief of its position for every waypoint. Only the top-right waypoint is generated from Bayes filter. We can see that due to some errors from the Baye's prediction, the robot over-corrects and misses the second to last target. 
[![alt text](lab12/run.png)]("Robot's percieved position")


</details>