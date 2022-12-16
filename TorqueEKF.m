%position = prevPos + vel*dt + accel*dt^2/2
%velocity = prevVel + accel*dt^2/2
%acceleration = engTorque*gearRatio/(wheelRadius * vehicle mass) ...
%... - dragCoeff*Velocity^2 /vehicle mass
%torque = prevTorque 
%gearRatio = gearRatio - provided from CANBUS or estimated...
%if estimating gear ratio... ratio = engineRPM/wheelVelocity...
%then need to convert wheelVel from velocity and also add RPM as a state...
clear;
load DriveData.mat
time = driveData.time;
dt = driveData.dt;
distance = driveData.GPSposition;
velocity = driveData.velocity;
acceleration = driveData.acceleration;
gearRatio = driveData.gearRatio;
RPM = driveData.RPM;

wheelRad = 0.36195; %radius of wheels in meters
m = 2100;%vehicle mass in kg
Xmeas = [distance,velocity,acceleration,gearRatio,RPM];

Q = [
10 0 0 0 0 0 0; 
0 3 0 0 0 0 0; 
0 0 4.0 0 0 0 0;
0 0 0 1 0 0 0;
0 0 0 0 0.5 0 0; 
0 0 0 0 0 10 0;
0 0 0 0 0 0 20;
];

R = [10 0 0 0 0; 0 2.0 0 0 0; 0 0 5 0 0; 0 0 0 0 0.5; 0 0 0 0 0.0];
G = [1 0 0 0 0 0 0; 0 1 0 0 0 0 0;0 0 1 0 0 0 0; 0 0 0 0 0 1 0; 0 0 0 0 0 0 1;];
inputs = zeros(length(time),1);
models = @(stateVec,inputVec) [
  stateVec(1) + stateVec(2)*dt + stateVec(3)*dt^2 /2;%position
  stateVec(2) + stateVec(3)*dt;%velocity
  stateVec(4)*stateVec(6)/(m*wheelRad) - stateVec(5)*stateVec(2)^2/m; %acceleration
  stateVec(4);%engine torque
  0.973;%dragCoefficient (includes air density, cross section area... etc.)% was going  to be variable... but for now a constant.
  stateVec(7)*pi*wheelRad/(30*stateVec(2));%convert wheel speed to rpm... compare...
  stateVec(7);%engine RPM%will rely exclusively on measured data...
];

F = @(stateVec,inputVec) [
1, dt, dt^2/2, 0, 0, 0,0;
0, 1, dt,0,0,0,0;
0,-2*stateVec(5)*stateVec(2)/m, 0, stateVec(6)/(m*wheelRad), stateVec(2)^2 /m, stateVec(4)/(m*wheelRad),0;
0,0,0,1,0,0,0;
0,0,0,0,1,0,0;
0, -stateVec(7)*pi*wheelRad/(30*stateVec(2).^2),0,0,0,0,pi*wheelRad/(30*stateVec(2));
0,0,0,0,0,0,1;
];

Xk = [1200; 4; 1;100;0.75;10;800;];%initial guesses... 
Pk = eye(7);
stateEstimates = EKF(models,inputs,F, G,Xk,Q,R,Pk,time,Xmeas);
figure(1)
subplot(6,1,1)
plot(time,distance,time,stateEstimates(:,1));
title("Distance from Start Position (m)")
legend("Experimental", "EKF Estimate")
subplot(6,1,2)
plot(time,velocity,time,stateEstimates(:,2));
title("Velocity (m/s)")
subplot(6,1,3)
plot(time,acceleration,time,stateEstimates(:,3));
title("Acceleration (m/s^2)")
subplot(6,1,4)
plot(time,stateEstimates(:,4));
title("Kalman Estimated Torque (N-m)")
subplot(6,1,5)
plot(time,gearRatio,time,stateEstimates(:,6));
title("Gear Ratio")
subplot(6,1,6)
plot(time,RPM,time,stateEstimates(:,7));
title("Engine RPM")
xlabel("Time (sec)")


%before tuning EKF like crazy... see if their are connections between explanatory variables...
X = [ones(length(time),1), velocity.^2, gearRatio, RPM];
Z = acceleration;
coeffs = OLS(X,Z);
figure(2)
subplot(3,1,1)
plot(time,Z,time,X*coeffs);
r2 = (coeffs.'*X.'*Z - length(Z)*mean(Z)^2)/(Z.'*Z - length(Z)*mean(Z)^2);
title("Acceleration Plot")
ylabel("Acceleration (m/s^2)")
xlabel("Time")
legend("Experimental", "Regressed")
%%residual analysis now...
subplot(3,1,2)
plot(time,Z - X*coeffs,'LineStyle', 'none', 'marker','.');
title("Residual vs Time");
xlabel("Time")
ylabel("Res")
subplot(3,1,3)
plot(X*coeffs,Z-X*coeffs, 'LineStyle', 'none', 'marker','.');
title("Residual vs Model Value")
xlabel("Model")
ylabel("Res")
[nZ,nR] = normReg(X,Z);
norm = OLS(nR,nZ);


torque = Z./gearRatio;
%plot(RPM,500*torque+00);
hold on
data2 = load("dynoData.csv");
r = data2(:,1);
t = data2(:,2);
%plot(r,t);

notable = find(acceleration>0.15);
%notable = find(0.2>driveData.throttle>0.15);
RPM = RPM(notable);
throttle = driveData.throttle(notable);
figure(69)
%plot(RPM(notable),3*stateEstimates(notable,4),'marker','o','LineStyle','none');
plot(r,t);
xlabel("RPM")
ylabel("Torque (N-m)")
title("Discovery II 4.6L V8 Torque Curve")


  