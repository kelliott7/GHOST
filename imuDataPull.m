%a = arduino('/dev/cu.usbmodem14101', 'Uno', 'Libraries', 'I2C');
%fs = 100;
%imu = mpu9250(a, 'SampleRate', fs, 'OutputFormat', 'matrix');

function [imu_orientation] = imuDataPull(imu)
% imuDataPull.m pulls sensor data from the IMU
%   Activates timer to call data from gyroscope while it is on
%   Evan Baker 05/26/2021
%   evan.baker@hilltechnicalsolutions.com

GyroscopeNoiseMPU9250 = 3.0462e-06;
AccelerometerNoiseMPU9250 = 0.0061;
stop_Timer = 100;

FUSE = ahrsfilter('SampleRate',imu.SampleRate, 'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise',AccelerometerNoiseMPU9250);

%Actual code to read data

tic;
while(toc < stop_Timer)
    [imu_accel,imu_angVel,imu_magField] = readSensorDataMPU9250(imu);
    imu_orientation = FUSE(imu_accel,imu_angVel,imu_magField);
    
end

end 



