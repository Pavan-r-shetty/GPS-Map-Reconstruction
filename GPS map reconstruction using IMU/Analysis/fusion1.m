close all % 3rd question code is in the end part of the code
clc

load imuraw.mat  %loading the files
load gps.mat
e = gps( :,6);
n = gps( :,7);
a = gps( :,5);
alt = table2array(a)
east = table2array(e)
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
gyrox = table2array(gx)  %converting table into arrays
gyroy = table2array(gy)
gyroz = table2array(gz)
linx = table2array(lx)
liny = table2array(ly)
linz = table2array(lz)
roll = table2array(r)
pitch = table2array(p)
yaw = table2array(y)
magx = table2array(mx)
magy = table2array(my)
magz = table2array(mz)
mag_x = magx(8000:10000)
mag_y = magy(8000:10000)



%plot(east,north)
%figure
%plot(magx)
%figure
%plot(magy)
%figure
%pl%Hard iron factor elimination
alpha = (max(mag_x)+min(mag_x))/2;
beta = (max(mag_y)+min(mag_y))/2;

hard_corrected_mag_value = [(mag_x-alpha), (mag_y-beta)];  %hard iron correction

%Soft iron factor elimination
temp_mag_value = cat(2,hard_corrected_mag_value(:,1),hard_corrected_mag_value(:,2));
[z,a,b,al] = fitellipse(temp_mag_value);
R = [cosd(al), -sind(al); sind(al), cosd(al)];
rotated_mag_temp = temp_mag_value*R;
gamma = a/b;
rotated_mag = rotated_mag_temp;
rot_mag_x = rotated_mag(:,1)
rot_mag_y = rotated_mag(:,2)
soft_corrected_mag_x = rot_mag_x/gamma    %soft iron correction
soft_corrected_mag_y = rot_mag_y





%% 

figure;
scatter(mag_x,mag_y,"red")
xlabel("Original Mag-x data")
ylabel("Original Mag-y data")
title('Original Magnatometer Data')
grid on
daspect([1 1 1]);

figure;
scatter(hard_corrected_mag_value(:,1),hard_corrected_mag_value(:,2))
xlabel("Hard Iron corrected Mag-X")
ylabel("Hard Iron corrected Mag-y")
title('Hard Iron Corrected Data')
grid on
daspect([1 1 1])

figure;
scatter(soft_corrected_mag_x,soft_corrected_mag_y,"green")
xlabel("soft-iron corrected mag-x")
ylabel("soft iron corrected mag-y")
title("soft iron corrected")
grid on
daspect([1 1 1])





%% 
%Calculate the yaw angle from the corrected magnetometer readings.
hard_corrected_mag_value = [(magx-alpha), (magy-beta)];

%Soft iron factor elimination
temp_mag_value = cat(2,hard_corrected_mag_value(:,1),hard_corrected_mag_value(:,2));

rotated_mag_temp = temp_mag_value*R;
gamma = a/b;
rotated_mag = rotated_mag_temp';
rot_mag_x = rotated_mag(1,:)
rot_mag_y = rotated_mag(2,:)
soft_corrected_mag_x = rot_mag_x/gamma
soft_corrected_mag_y = rot_mag_y           %correction for the entire data
%% 

yawc = atan2(soft_corrected_mag_y,soft_corrected_mag_x)   %yaw from corrected mag values



yawct = yawc(13000:51247);   %eliminination of the initial rest position values for better comparison
Fs = 40;
t0 = 1/Fs;
%% 

yawintz = (cumtrapz(gyroz,1)*t0)+35;  %yaw by integrating gyroz and bias removal because of the integration
yawtemp = yawintz';
yawt = yawtemp(13000:51247);
figure
plot(yawt,'r')
hold on
plot(yawct,'b')
hold off

xlabel("time")
ylabel("yaw")
legend('yaw from gyro','yaw from magnetometer')
title("yaw comparison from the two methods")
grid on

%% 


figure
lowpas = lowpass(yawct,.01,40)
highpas = highpass(yawt,.01,40)
comp = lowpas+highpas


%comp = 0.5*yawct + 0.5*yawt     %complimentary filter
plot(comp,'r')
xlabel("time")
ylabel("yaw")

title("complimentary filter on yaw from the two methods")
grid on

figure()

plot((yaw(13000:51247)/1.5),'r')
xlabel("time")
ylabel("yaw")

title("yaw from raw imu")
grid on



%% 3rd question
figure()
plot(linx)
meanlinx = mean(linx);  %correction of forward acceleration before integrating to remove the bias
coraccx = linx-meanlinx;
for i = 1:length(coraccx)
    if coraccx(i)<0
        coraccx(i) = 0;
    end
end
velx = coraccx.*unwrap(cos(yawc))';
vely = coraccx.*unwrap(sin(-yawc))';

figure()
xx = cumtrapz(velx);
xy = cumtrapz(vely);
plot(xx,xy)
xlabel("eastin")
ylabel("northing")
title("displacement from corrected acceleration of imu")



figure
plot(east,north)
xlabel("eastin")
ylabel("northing")
title("GPS Plot")

%% 
figure
Xdot = cumtrapz(linx);
wXdot = gyroz.*Xdot./yaw; %divided with yaw for dimensional accuracy
plot(wXdot)
xlabel("time")
ylabel("omega times xdot according to the formula")
title("ydotdot according to the formula")
figure()
plot(liny)
xlabel("time")
ylabel("acceleration in y")
title("imu acceleration in y")
%% 
xc = (liny - wXdot)./gyroz



