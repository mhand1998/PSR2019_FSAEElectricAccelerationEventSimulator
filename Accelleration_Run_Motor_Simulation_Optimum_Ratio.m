%2019 Electric Car Accelleration Gear Optimizer 
%Started: 3/5/17

clc
close
clear

%motor parameters
motor = csvread('Emrax 208 Torque vs. Speed.csv');
%motor(:,2) = motor(:,2)*100;

%drivetrain parameters
drivetrain_effeiency = .9;
gear_ratio_max = 11; 
gear_ratio_min = 9;
test_interval = .05;

%vehicle parameters
m = 320;%[kg]
g = 9.8;    %[m/s^2]
wdia = 0.46482; %[m]
wb = 1.6; %[m]
CGz = .2;        %[m]
swd = .55;   %[] weight distribution (rearward bias)
ld = .55;    %[] lift distripbution (reward bias)
mu = 1.57;    %[] longitudinal
cd = 0.860;   %[] drag coefficent
cl = 1.900;     %[] lift coefficent
ref_area = 0.527;    %[m^2] refference area
air_density = 1.225; %[kg/m^3]
rolling_resistance = .015;  %[]


%pre-allocate arrays
s = zeros(1,3);  %[m] position
v = zeros(1,3);  %[m/s] velocity
a = zeros(1,3);  %[m/s^2] acceleation
t = zeros(1,3);  %[seconds] time

gear_ratio_array = [gear_ratio_min:test_interval:gear_ratio_max];

t_increment = .0001;

for j = 1:numel(gear_ratio_array)

i = 1;
%rps = 0;
%rpm = 0;
%motor_rpm = zeros(1,1);
%motor_torque = zeros(1,1);
%f_motor = 0;
%f_wheel_actual = zeros(1,1);
%f_wheel_max = 0;

wcir = wdia*2*pi;

while s(1,i)<=75
rps = v(1,i)/wcir;
rpm = rps*60;
motor_rpm(1,i) = rpm*gear_ratio_array(1,j);

rpm_idx = Closest(motor(:,1),motor_rpm(1,i));

motor_torque(1,i) = motor(rpm_idx,2);
f_motor = motor_torque(1,i)*gear_ratio_array(1,j)/(wdia/2)*drivetrain_effeiency;

    
f_drag = Drag(cd,ref_area,air_density,v(1,(i))); %[N]
f_lift = Lift(cl,ref_area,air_density,v(1,(i))); %[N}

a_max = g*mu*swd/(1-mu*CGz/wb)+(f_lift*ld*mu/m); %[m/s^2]
f_wheel_max = m*g*mu*swd/(1-mu*CGz/wb)+(f_lift*ld*mu); %[N(kg m/s^2)]
f_rolling_resistance = (m*g+f_lift)*rolling_resistance;

if f_motor>=f_wheel_max
    f_wheel_actual(1,i) = f_wheel_max;
else
    f_wheel_actual(1,i) = f_motor;
end

f_net(1,i) = f_wheel_actual(1,i)-f_drag-f_rolling_resistance;

a(1,i) = f_net(1,i)/m;
v(1,(i+1)) = v(1,i) + a(1,i)*t_increment;
s(1,(i+2)) = s(1,i+1) + v(1,(i+1))*t_increment;
t(1,i) = i*t_increment;

i = i+1;
end
accel_time = t(1,(i-1));
times(1,j) = accel_time;
end
[fastest,idx] = min(times);
optimum_ratio = gear_ratio_array(1,idx);

%make arrays equal length for plots
i = i-1;
a(1,(i+1:i+2)) = a(1,i);
t(1,(i+1:i+2)) = t(1,i);
v(1,(i+1:i+2)) = v(1,(i+1));
f_wheel_actual(1,(i+1:i+2)) = f_wheel_actual(1,i);

fprintf('Fastest Accel Time: %.3f seconds\n' ,fastest);
fprintf('Optimum Gear Ratio: %.2f\n' ,optimum_ratio)

plot(gear_ratio_array,times)

%{
subplot(2,2,1)
plot(t,v)
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
title('Velocity vs. Time')
subplot(2,2,2)
plot(t,a)
xlabel('Time (sec)')
ylabel('Acceleration (m/s^2')
title('Acceleration vs. Time')
subplot(2,2,3)
plot(t,f_wheel_actual)
xlabel('Time (sec)')
ylabel('Wheel Force (N)')
title('Wheel Force vs. Time')
%}