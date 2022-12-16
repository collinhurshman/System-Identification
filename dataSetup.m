data = dlmread("SIDDataFullRouteRun3.csv",',');
time = data(:,1)/1000;
%dt not perfectly uniform, roughly 30ms... averaging
diff = zeros(length(time)-1,1);
for i =1:length(time)-1
  diff(i) =(time(i+1) - time(i));
end
dt = mean(diff);
%gps wasn't initialized until sample 34...
data(1:33,2) = data(34,2);
data(1:33,3) = data(34,3);

lat = data(:,2)*pi/180;%convert to radians
lon = data(:,3)*pi/180;
R = 6371*10^3; %radius of earth
x = R.*cos(lat).*cos(lon);
y = R.*cos(lat).*sin(lon);

%plotting out the route for reference
figure(1);
plot(x,y,'linewidth',5,'color','g')
title("Vehicle Test Route")
xticks([]);
yticks([]);

%computing distance traveled
distance = zeros(length(time),1);
for i=2:1:length(time)
  %using original lat,lon... rad conversion done in earthdist function
  pastPos = [data(i-1,3),data(i-1,2)];
  currentPos = [data(i,3),data(i,2)];
  distance(i) = distance(i-1) + earthDistance(pastPos,currentPos);
end

%setting up acceleration & velocity from gps
%GPS MODULE REPORTS AT ROUGHLY 1HZ... HAVE TO INTERPOLATE POSITIONS TO SMOOTH OUT REPEAT READINGS
interpolated = zeros(length(distance),1);
current = distance(1);
index = 1;
for i = 1:length(distance)%find dissimilar gps readings... interpolate between them
  if(distance(i) ~= current)
   interpolated(i) = distance(i);
   current = distance(i); 
  end
end
timeAdj = time(2:length(time));%keep initial entries
intAdj = interpolated(2:length(time));
idx = find(interpolated ~= 0);
interpolated = interp1(time(idx),interpolated(idx),time,'spline');
idx = find(isnan(interpolated));
interpolated(idx) = 0;%first several datapoints can't be interpolated...
%last datapoints are zeroed out too...
interpolated(22155:length(interpolated)) = interpolated(22154);
gpsVel = deriv(interpolated,dt);
gpsVel = smooth(gpsVel,40);
gpsAccel = deriv(gpsVel,dt);
%STRUGGLING TO GET GOOD GPS DISTANCE & VELOCITY... LEAVING IT FOR NOW

rpm = smooth(data(:,5),40);
wheelVel = smooth(data(:,4),40)/3.6;%meters per second

figure(2)
ax = plotyy(time,rpm,time,wheelVel);
ylabel (ax(1), "Engine Speed (RPM)");
ylabel (ax(2), "Road Speed (m/s)");
xlabel("Time (sec)")

wheelRad = 0.36195; %radius of wheels in meters

%checking out gear ratios
wVec = 1:0.1:5;
rpmdt = rpm - mean(rpm);
zdt = (wheelVel*1000/60) - mean(wheelVel)*1000/60;
X = [ones(length(time),1), rpm];
[xf,zf,rtc] = RFT(time,wVec,X,wheelVel*1000/60,0.956);
%converting wheelVel to m/s...

figure(3)
subplot(2,1,1)
plot(time,rtc);
subplot(2,1,2)
plot(time,data(:,6));

%figure(4)
wheelAcc = deriv(wheelVel,dt);
%wheelAcc = smooth(wheelAcc,10);
plot(time,wheelAcc);

ratioList = zeros(length(time),1);
%inserting manufacturer specified gear ratios...
for i = 10:length(ratioList)
  switch (data(i,6))
    case (1)
      ratio = 2.48*1.211*3.538;
    case (2)
      ratio = 1.48*1.211*3.538;
    case (3)
      ratio = 1.0*1.211*3.538;
    case (4)
      ratio = 0.728*1.211*3.538;
    otherwise%use previous gear ratio if in between..
      ratio = ratioList(i-1);
  end
    ratioList(i) = ratio;
end

%TEST OLS!!!
Z = wheelAcc;
X = [ones(length(wheelAcc),1), rpm, wheelVel, ratioList];
coeffs = OLS(X,Z);
figure(69)
plot(time,Z,time,X*coeffs);
legend("Numerical Acceleration","OLS Acceleration");
r2 = (coeffs.'*X.'*Z - length(Z)*mean(Z)^2)/(Z.'*Z - length(Z)*mean(Z)^2);
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")

%now working with OBD data...
OBD = load("Run3OBDDataThrottle.csv");
OBDt = OBD(:,1)-41470 - 90;%just time shifting data...
throttle = OBD(:,2)./100;%as decimal rather than percent
OBDRPM = smooth(OBD(:,3),10);
figure(4)
plot(OBDt,OBDRPM);
hold on
plot(time,rpm);

%space throttle data out better with interpolation...
diff = zeros(length(OBDt)-1,1);
for i =1:length(OBDt)-1
  diff(i) =(OBDt(i+1) - OBDt(i));
end
OBDdt = mean(diff);%horrific sampling time of 1.33 seconds
step = round(OBDdt/dt);
interpolated = zeros(length(time),1);
current = throttle(1);
index = 1;
for i = 1:length(throttle)%put slow throttle readings at correct times...
  [d, ix] = min(abs(OBDt(i)-time));%find closest time matchup
  interpolated(ix) = throttle(i);
end
idx = find(interpolated ~= 0);
interpolated = interp1(time(idx),interpolated(idx),time,'spline');
idx = find(isnan(interpolated));
interpolated(idx) = 0;%first several datapoints can't be interpolated...

startIDX = 5500;%setting up interval of interest...
endIDX =10000;
figure(5)
title("Shortened Route Traveled")
plot(x(startIDX:endIDX),y(startIDX:endIDX));
driveData.throttle = interpolated(startIDX:endIDX);
driveData.dt = dt;
driveData.time = time(startIDX:endIDX);
driveData.GPSposition = distance(startIDX:endIDX); %for gps distance
distance = cumtrapz(dt,wheelVel);%or for wheel velocity based distance
driveData.position = distance(startIDX:endIDX);
driveData.velocity = wheelVel(startIDX:endIDX);
driveData.acceleration = wheelAcc(startIDX:endIDX);
driveData.gearRatio = ratioList(startIDX:endIDX);
driveData.RPM = rpm(startIDX:endIDX);
save("DriveData.mat", 'driveData');%save drivedata struct to work with later..