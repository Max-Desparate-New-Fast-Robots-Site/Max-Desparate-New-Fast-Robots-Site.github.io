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

<details>

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

# Applying Kalman Filter
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

## Onboard extrapolation and Kalman filter
To interpolate between TOF readings on the Artemis, I use my last Kalman-filtered readings of x and xdot to linearly extrapolate into the future. This allows me to have a finer sampling rate. 

I use the following function to update my globally-stored mean and sigma matrices. 
```C
float kf(float u, float y){

  // prediction
  Matrix<2,1> mu_p = A*x_ + B*u;
  Matrix<2,2> sig_p = A*sig*(~A) + sig_u;

  // update
  Matrix<1,1> sig_m = C*sig_p*(~C) + sig_z;
  Matrix<1,1> sig_m_inv = sig_m;
  Invert(sig_m_inv);
  Matrix<2,1> kkf_gain = sig_p*(~C*sig_m_inv);

  Matrix<1,1> y_curr = {dist};
  Matrix<1,1> y_m = y_curr - C* mu_p;

  x_ = mu_p + kf_gain*y_m;
  sigma = (Eye - kkf_gain*C)*sig_p;

  return x_(0,0);
}
```

And after capturing the initial state with the TOF sensor, I am able to begin applying the filter to incoming data. In this code, when available, I pass the TOF data through the Kalman filter and store that as my predicted state. Otherwise, every 15 milliseconds, at a much faster sampling rate, I use my best guess of my real position and speed to predict the present state. 

```C
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
            xdot = (distance1 - buff0[counter-1]) / dt;
        }
        
        y = {distance1,xdot};
        kf(u,-y);

        times[counter] = timestamp/1000.;
        distance1 = y(0,0);
        buff0[counter] = distance1;
        buff1[counter] = y(0,1);
        counter++;
        }

        delay(15);
        times[counter] = timestamp/1000.;
        distance1 += 15*y(0,1);
        buff0[counter] = distance1;
        buff1[counter] = y(0,1);
        counter++;

}
```

</details>