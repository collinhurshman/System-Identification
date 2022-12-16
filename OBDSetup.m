clear;
data = dlmread("StraightOBD.csv",',');
data = data(7:length(data),:);
time = data(:,1);
%dt may not be not perfectly uniform, roughly 30ms... averaging
diff = zeros(length(time)-1,1);
for i =1:length(time)-1
  diff(i) =(time(i+1) - time(i));
end
dt = mean(diff);
rpm = data(:,2);
load = data(:,4);
speed = data(:,5);
airFlow = data(:,6);
throttle = data(:,7);

accel = deriv(speed,dt);
plot(time,accel);
X = [ones(length(time),1),rpm,load];
Z = accel;
coeffs = OLS(X,Z);
plot(time,accel,time,X*coeffs)
legend("accel", "model")
