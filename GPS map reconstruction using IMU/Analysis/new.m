close all %first and 2nd question code, 3rd question code is in the end part of the fusion.m code
clc

load imuraw.mat  %loading the files
load gps.mat
e = gps( :,6);
n = gps( :,7);
a = gps( :,5);
alt = table2array(a);
east = table2array(e);
north = table2array(n)
r = imuraw( :,2);
p = imuraw( :,3);
y = imuraw( :,4);

mx = imuraw( :,5);
my = imuraw( :,6);
mz = imuraw( :,7);
lx = imuraw( :,8);
ly = imuraw( :,9);
lz = imuraw( :,10);
gx = imuraw( :,11);
gy = imuraw( :,12);
gz = imuraw( :,13);
gyrox = table2array(gx); %converting table into arrays
gyroy = table2array(gy);
gyroz = table2array(gz);
linx = table2array(lx);
liny = table2array(ly);
linz = table2array(lz);
roll = table2array(r);
pitch = table2array(p);
yaw = table2array(y);
magx = table2array(mx);
magy = table2array(my);
magz = table2array(mz);
mag_x = magx(8000:10000);
mag_y = magy(8000:10000);
%% 
alpha = (max(mag_x)+min(mag_x))/2;
beta = (max(mag_y)+min(mag_y))/2;

figure;
scatter(mag_x,mag_y,"red")
xlabel("Original Mag-x data")
ylabel("Original Mag-y data")
title('Original Magnatometer Data')
grid on
daspect([1 1 1]);


hard_corrected_mag_value = [(mag_x-alpha), (mag_y-beta)];  %hard iron correction
figure;
scatter(hard_corrected_mag_value(:,1),hard_corrected_mag_value(:,2))
xlabel("Hard Iron corrected Mag-X")
ylabel("Hard Iron corrected Mag-y")
title('Hard Iron Corrected Data')
grid on
daspect([1 1 1])
%% 
%Soft iron factor elimination
hardx = hard_corrected_mag_value(:,1);
hardy = hard_corrected_mag_value(:,2);
r = sqrt(hardx.^2 + hardy.^2);
%% 
[r_v,r_I] = max(r);
[m_v,m_I] = min(r);
theta = asind(hard_corrected_mag_value(r_I,2)/r_v);
%% 

R = [cos(theta),sin(theta); -sin(theta),cos(theta)];
mag_corrected_values_real = R*hard_corrected_mag_value';

%% 

scale_factor = m_v/r_v;
mag_corrected_values_real(1,:) = mag_corrected_values_real(1,:)*scale_factor;
%% 
theta = -theta;
R = [cosd(theta) sind(theta); -sind(theta) cosd(theta)];
mag_corrected_values_real = R*mag_corrected_values_real;
%% 

figure
axis equal; 
axis square;
scatter(mag_corrected_values_real(1,:),mag_corrected_values_real(2,:));
title(' after adjustments for Soft and Hard Iron effects'); 
xlabel('X-readings');
ylabel('Y-readings');
%% 

mag_corrected_values_real = mag_corrected_values_real';
soft_corrected_mag_y = mag_corrected_values_real(:,2);
soft_corrected_mag_x = mag_corrected_values_real(:,1);


yawtemp = atan2(soft_corrected_mag_y,soft_corrected_mag_x)   %yaw from corrected mag values




%%

magx2 = magx(13200:51240);
magy2 = magy(13200:51240);
hmagx = magx2 - alpha;
hmagy = magy2 - beta;
hmag = [hmagx,hmagy];
smag = hmag*R;
yawfrommag = atan2(smag(:,2),smag(:,1))     %yaw for the whole data from mag


%% 
gyro = gyroz(13200:51240);               %yaw from gyro
yawfromgyro = cumtrapz(gyro);

figure()

plot(yawfromgyro,'r')
hold on

plot(yawfrommag, 'b')

xlabel("time")
ylabel("yaw")
legend('yaw from gyro','yaw from magnetometer')
title("yaw comparison from the two methods")
grid on
%% 


figure

comp =yawfrommag*0.99 + yawfromgyro*0.01        %complementary filter
plot(comp-0.4,'r')
hold on
yawreq = (yaw(13200:51240)-2.4);
plot(yawreq,'b')
xlabel("time")
ylabel("yaw")
legend('yaw from imu','yaw from filter')
title("yaw from raw imu vs complementary filter")
grid on


%% 
%2nd question
figure()
linx;  
plot(linx)
xlabel("time")
ylabel("forward acceleration")

title("forward acceleration before correction")


figure()
forwardaccel = linx(13200:51240)-mean(linx(13200:51240));
plot(forwardaccel)
xlabel("time")
ylabel("forward acceleration")

title("forward acceleration after correction")
fvel = cumtrapz(forwardaccel)

figure()
plot(fvel)  
xlabel("time")
ylabel("forward velovity")

title("forward velocity from acceleration")
%% 
figure()
northing = north(330:1281);
easting = east(330:1281)
dVx = gradient(northing);  %calculation for differentiating easting and northing of gps data
dVy = gradient(easting);

dt = 1; % sec.
vx = dVx./dt;
vy = dVy./dt;
vel = sqrt(vx.^2 + vy.^2)  %resulatnt velocity from x and y components
plot(vel)
xlabel("time")
ylabel("velocity")
%legend('yaw from gyro','yaw from magnetometer')
title("velocity plot from gps data")
grid on

%% 
a = northing;        %scaling of gps to match imu frequency
b = a';
l = 1;
for i= 1:length(b)
    j = i;
    for k = 1:40
        c(l) = b(j);
        l = l + 1;
    end
end
%% 
aa = easting;        %scaling of gps to match imu frequency
bb = aa';
ll = 1;
for ii= 1:length(bb)
    jj = ii;
    for kk = 1:40
        cc(ll) = bb(jj);
        ll = ll+ 1;
    end
end
utmeasting = cc(1:38041);
utmnorthing = c(1:38041);
%% 

figure()
plot(forwardaccel)
title("accelerataion after correction")
xlabel("time")
ylabel("forward acceleration")
%% 

fvel1 = cumtrapz(forwardaccel)

%dVx1 = gradient(utmeasting);  %calculation for differentiating easting and northing of gps data
%dVy1 = gradient(utmnorthing);

%dt1 = 1; % sec.
%vx1 = dVx1'./dt1;
%vy1= dVy1'./dt1;
%vx1 = diff(utmeasting);
%vy1 = diff(utmnorthing);
%vel1 = sqrt(vx1.^2 + vy1.^2)  %resulatnt velocity from x and y components
%% 


figure()
plot(fvel1,'r')  
%hold on
%plot(vel,'b')

xlabel("time")
ylabel("forward velocity")
%% 
%xcomp = fvel1.*(cos(deg2rad(yawreq)));
%ycomp = fvel1.*(sin(deg2rad(yawreq)));
%xxx = cumtrapz(xcomp);
%yyy = cumtrapz(ycomp);
%plot(xxx,yyy)











 

