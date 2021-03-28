# Quad_Estimator
Udacity Autonomous Flight Nanodegree - Estimator Project


## Step 1 - Sensor Noise

```
import numpy as np

filename = 'Graph1.txt'
GPS_X_data = np.loadtxt(filename, delimiter = ',', dtype = 'Float64', skiprows = 1)

GPS_X_mean = 0
for i in range(len(GPS_X_data[:,1])):
    GPS_X_mean += GPS_X_data[i][1]
GPS_X_mean = GPS_X_mean/len(GPS_X_data[:,1])

GPS_X_variance = 0
for i in range(len(GPS_X_data[:,1])):
    GPS_X_variance += (GPS_X_mean - GPS_X_data[i][1])**2
GPS_X_variance = GPS_X_variance / len(GPS_X_data[:,1])
GPS_X_std_dev = np.sqrt(GPS_X_variance)

print('GPS X Mean = ',GPS_X_mean)
print('GPS X variance = ',GPS_X_variance)
print('GPS X standard deviation = ',GPS_X_std_dev)
```


```
filename = 'Graph2.txt'

IMU_AX_data = np.loadtxt(filename, delimiter = ',', dtype = 'Float64', skiprows = 1)

IMU_AX_mean = 0
for i in range(len(IMU_AX_data[:,1])):
    IMU_AX_mean += IMU_AX_data[i][1]
IMU_AX_mean = IMU_AX_mean/len(IMU_AX_data[:,1])

IMU_AX_variance = 0
for i in range(len(IMU_AX_data[:,1])):
    IMU_AX_variance += (IMU_AX_mean - IMU_AX_data[i][1])**2
IMU_AX_variance = IMU_AX_variance / len(IMU_AX_data[:,1])
IMU_AX_std_dev = np.sqrt(IMU_AX_variance)

print('IMU AX Mean = ',IMU_AX_mean)
print('IMU AX variance = ',IMU_AX_variance)
print('IMU AX standard deviation = ',IMU_AX_std_dev)
```



## Step 2 - Attitude Estimation

Implemented according to section 7.1.2 Non-Linear Complementary Filter from the paper "Estimation for Quadrotors.

QuadEstimatorEKF.cpp 
function UpdateFromIMU()
 
```
float z_phi_dot = gyro.x + sin(rollEst)*tan(pitchEst)*gyro.y + cos(rollEst)*tan(pitchEst)*gyro.z;
float z_theta_dot = cos(rollEst)*gyro.y - sin(rollEst)*gyro.z;    
float z_psi_dot = (sin(rollEst)/cos(pitchEst))*gyro.y + (cos(rollEst)/cos(pitchEst))*gyro.z;
```


```
float predictedPitch = pitchEst + dtIMU*z_theta_dot;
float predictedRoll = rollEst + dtIMU*z_phi_dot;
ekfState(6) = ekfState(6) + dtIMU * z_psi_dot;    // yaw
```

```
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
```


```
// CALCULATE UPDATE
accelRoll = atan2f(accel.y, accel.z);
accelPitch = atan2f(-accel.x, 9.81f);
```

```
// FUSE INTEGRATION AND UPDATE
rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;
```


## Step 3 - Prediction Step

QuadEstimatorEKF.cpp file
PredictStateFunction() function

```
const double CONST_GRAVITY = 9.81; // gravity in [m/s^2]
    
V3F Inertial = attitude.Rotate_BtoI(accel);
    
predictedState[0] = curState[0] + curState[3]*dt;
predictedState[1] = curState[1] + curState[4]*dt;
predictedState[2] = curState[2] + curState[5]*dt;
predictedState[3] = curState[3] + Inertial.x*dt;
predictedState[4] = curState[4] + Inertial.y*dt;
predictedState[5] = curState[5] + Inertial.z*dt - CONST_GRAVITY*dt;
predictedState[6] = curState[6];
```


## Step 4 - Magnetometer Update

## Step 5 - Closed Loop and GPS Update

## Step 6 - Adding my Controller
