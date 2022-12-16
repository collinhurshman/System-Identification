%position = prevPos + vel*dt + accel*dt^2/2
%velocity = prevVel + accel*dt^2/2
%acceleration = engTorque*gearRatio/(wheelRadius * vehicle mass) ...
%... - dragCoeff*Velocity^2 /vehicle mass
%torque = prevTorque %ASSUMES CONSTANT TORQUE... WILL DEFINITELY NEED ADJUSTMENT?
%gearRatio = gearRatio - provided from CANBUS or estimated...
%if estimating gear ratio... ratio = engineRPM/wheelVelocity...
%then need to convert wheelVel from velocity and also add RPM as a state...
clear;
load("DriveData.mat")
time = driveData.time;
dt = driveData.dt;
distance = driveData.GPSposition;
velocity = driveData.velocity;
acceleration = driveData.acceleration;
gearRatio = driveData.gearRatio;
wheelRad = 0.36195; %radius of wheels in meters
m = 2100;%vehicle mass in kg
Xmeas = [distance, acceleration];
Q = [
10 0 0;
0 10 0;
0 0 10; 
];
R = [1 0; 0 1;];
G =  [1 0 0; 0 0 1];
inputs = zeros(length(time),1);
models = @(stateVec,inputVec) [
  stateVec(1) + stateVec(2)*dt;%position
  stateVec(2);
];

%OR ARE THE Q AND R MATRICES JUST POORLY TUNED???
F = @(stateVec,inputVec) [
1, dt;
0, 1;
];

figure(1)
Xk = [1200; 20; 0;];%initial guesses... 
Pk = 0.1*eye(3);
stateEstimates =  kalman([1 dt 0.5*dt^2; 0 1 dt; 0 0 1],[0; 0; 0;],0,G,Xk,Q,R,Pk,time,Xmeas); %EKF(models,inputs,F, G,Xk,Q,R,Pk,time,Xmeas);


est = trapz(dt,velocity);
act = distance(length(distance)) - distance(1);%EARTH DISTANCE FUNCTION MAY BE INACCURATE...
subplot(3,1,1)
plot(time,distance,time,stateEstimates(:,1));
subplot(3,1,2)
plot(time,velocity,time,stateEstimates(:,2));
subplot(3,1,3)
plot(time,acceleration,stateEstimates(:,3));

figure(2)
plot(time,velocity);



