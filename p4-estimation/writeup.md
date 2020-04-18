# Estimation Project - Writeup

### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. 

In order to process the measurement noise I took the data from `Graph1.txt` and `Graph2.txt` and extracted the standard 
deviation using the python code:

```Python
import csv
import sys
import math

with open("g2.txt") as f:
    reader = csv.reader(f)
    vals = []
    for row in reader:
        vals.append(row[1])
    vals = [float(i) for i in vals[1:]]

mean = float(sum(vals) / len(vals))

s=0
for i in vals:
    s = s + (i - mean) ** 2

print(math.sqrt(s / len(vals)))
```

The result:

```
MeasuredStdDev_GPSPosXY = 0.7221354406057051
MeasuredStdDev_AccelXY = 0.5131842554409878
```

### 2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.

We get the quaternion from the following:

```cpp
Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
qt.IntegrateBodyRate(gyro, dtIMU);
```

We can then get the predicted values from the quaternion:

```cpp
qt.Roll()
qt.Pitch()
```

### 3. Implement all of the elements of the prediction step for the estimator.

The `RBG'` matrix was calculated as follows:

```cpp
MatrixXf RbgPrime(3, 3);
RbgPrime.setZero();

// 1st row
RbgPrime(0,0) = - cos(pitch) * sin(yaw);
RbgPrime(0,1) = - sin(roll)  * sin(pitch) * sin(yaw) - cos(pitch) * cos(yaw);
RbgPrime(0,2) = - cos(roll)  * sin(pitch) * sin(yaw) + sin(roll)   * cos(yaw);
// 2nd row
RbgPrime(1,0) = cos(pitch) * cos(yaw);
RbgPrime(1,1) = sin(roll)  * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
RbgPrime(1,2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
```

`g'` was calculated as follows:

```cpp
MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
gPrime.setIdentity();

// create helper matrices
V3F rPBG[3];
for(int i=0; i<3; i++){
    rPBG[i] = V3F(RbgPrime(i,0), RbgPrime(i,1), RbgPrime(i,2));
}

// fill up gPrime matrix
gPrime(0,3) = dt;
gPrime(1,4) = dt;
gPrime(2,5) = dt;
gPrime(3, 6) = (rPBG[0] * accel).sum() * dt;
gPrime(4, 6) = (rPBG[1] * accel).sum() * dt;
gPrime(5, 6) = (rPBG[2] * accel).sum() * dt;
```

The updated (estimated) covariance was calculated using:

```cpp
ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

The `PredictState()` function makes a simple prediction by integrating the accelerations and velocities:

```cpp
// Update predicted x,y,z using simple linear integration
predictedState(0) = curState(0) + dt * curState(3);
predictedState(1) = curState(1) + dt * curState(4);
predictedState(2) = curState(2) + dt * curState(5);

// rotate accel from bodyframe to inertial frame
accel = attitude.Rotate_BtoI(accel);

// update predicted vx,vy,vz
predictedState(3) = curState(3) + dt * accel.x;
predictedState(4) = curState(4) + dt * accel.y;
predictedState(5) = curState(5) + dt * (accel.z - CONST_GRAVITY);
```

**Note:** The input accelerations are in the body frame so they are inverted to the inertial frame through the use of `attitude.Rotate_BtoI(accel)`.

### 4. Implement the magnetometer update.

The magnetometer is updated as follows:

```cpp
zFromX(0) = ekfState(6);
float diff = magYaw - zFromX(0);

// normalize diff to -pi .. pi
if (diff > F_PI) diff -= 2.f*F_PI;
if (diff < -F_PI) diff += 2.f*F_PI;

// update zFromX based on normalized diff
zFromX(0) = magYaw - diff;

// hPrime = [0,0,0,0,0,1]
hPrime(0, 6) = 1;
```

Note that the difference is normalised between `-π and π`.

### 5. Implement the GPS update.

The gps data is updated as follows:

```cpp
for (int i=0; i<6;i++){
    zFromX(i) = ekfState(i);
}

for (int i = 0; i < 6; i++){
    hPrime(i,i) = 1.f;
}
```

The estimated values were simple taken from `ekfState` as you can see above.

### 6. Meet the performance criteria of each step.

The project passes all performance criteria.

### 7. De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

I used my controller implementation from control project and managed to pass the performance criteria without any significant changes.
