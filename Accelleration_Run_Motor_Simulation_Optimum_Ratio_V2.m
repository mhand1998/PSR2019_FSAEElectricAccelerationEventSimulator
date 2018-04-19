%{
Electric Car Accelleration Event Simulation
Author:Michael Hand
Started: 3/5/17
This file is subject to the terms and conditions defined in file 'LICENSE.txt', which is part of this source code package.
%}


%%
clc
close
clear

%Load Vehicle Profile

[vehicleProfile,PathName] = uigetfile('','Load Vehcile Profile','*.mat');
load(fullfile(PathName,vehicleProfile))

mCar = m;
clear m;

%%
%Load Motor Profile

[motorProfile,PathName] = uigetfile('','Load Motor Profile','*.mat');
load(fullfile(PathName,motorProfile))

%%
%Set Up Optimisor
gear_ratio_array = [];

%gear_ratio_max = 3; 
%gear_ratio_min = 7;
%test_interval = .05;

gear_ratio_array = 3:.05:8; %min ratio: step size: max ratio


%%
close

drag_strip_length = 75; %[m} distance for acell run
times = [];

m = mCar+motorMass+controllerMass;

for j = 1:numel(gear_ratio_array)

%Setup
power_limited = false;
wcir = wdia*pi;
s = zeros(1,2);     %[m] position
v = 0;              %[m/s] velocity
vars = {'t','a','battery_draw_power','current','EMF','f_motor','f_net','f_total_drag','f_wheel_actual','f_wheel_max','motor_rpm','motor_torque','power',};
clear(vars{:})
t_increment = .001; %[sec] dt for simulation loop
i = 1;              %[] counter

gear_ratio = gear_ratio_array(1,j);

%Simulation Loop
%s = Position, v = velocity, a = acceleration

while s(1,i)<=drag_strip_length
rps = v(1,i)/wcir;
rpm = rps*60;
motor_rpm(1,i) = rpm*gear_ratio;


rpm_idx = Closest(motor(:,1),motor_rpm(1,i));

motor_torque(1,i) = motor(rpm_idx,2);
%motor_torque(1,i) = motor_fit(1,1)*(motor_rpm(1,i))^3+motor_fit(1,2)*(motor_rpm(1,i))^2+motor_fit(1,3)*motor_rpm(1,i)+motor_fit(1,4);
f_motor(1,i) = (motor_torque(1,i)/(wdia/2))*powertrain_effeiency*gear_ratio;

%Motor Electrical Calculations
current(1,i) = motor_torque(1,i)/torque_constant;
EMF(1,i) = (motor_rpm(1,i)/BackEMF_constant)+(current(1,i)*winding_resistance);
power(1,i) = EMF(1,i)*current(1,i); 
battery_draw_power(1,i) = power(1,i)*(1/inverter_efficency);
current_reduction = ((battery_draw_power(1,i)-80000)/EMF(1,i));
torque_reduction = current_reduction*torque_constant;

if battery_draw_power(1,i) < 80000
    f_motor(1,i) = (motor_torque(1,i)/(wdia/2))*powertrain_effeiency*gear_ratio;
else
    f_motor(1,i) = ((motor_torque(1,i)-torque_reduction)/(wdia/2))*powertrain_effeiency*gear_ratio;
    battery_draw_power(1,i) = battery_draw_power(1,i)-(current_reduction*EMF(1,i));
    power_limited = true; 
end

f_lift = Lift(cl,ref_area,air_density,v(1,(i))); %[N}
f_drag = Drag(cd,ref_area,air_density,v(1,(i))); %[N]
f_rolling_resistance = (m*g+f_lift)*rolling_resistance;
f_wheel_max(1,i) = m*g*mu*swd/(1-mu*CGz/wb)+(f_lift*ld*mu); %[N(kg m/s^2)]

if f_motor(1,i)>=f_wheel_max(1,i)
    f_wheel_actual(1,i) = f_wheel_max(1,i);
else
    f_wheel_actual(1,i) = f_motor(1,i);
end

f_total_drag(1,i) = f_drag+f_rolling_resistance;
f_net(1,i) = f_wheel_actual(1,i)-f_total_drag(1,i);

if motor_rpm(1,i) > peak_full_load_speed
a(1,i) = 0;
else
a(1,i) = f_net(1,i)/m;
end

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

fprintf('Fastest Accel Time: %.3f seconds\n' ,fastest);
fprintf('Optimum Gear Ratio: %.2f\n' ,optimum_ratio)

plot(gear_ratio_array,times)
xlabel('Gear Ratio')
ylabel('Acceleration Time (sec)')
title('Gear Ratio vs. Accell Time (Coef = 1.6)')

%%
% Append to Worksheet
filename = 'MotorOptimumRatio.xls';

time  = clock;
exportTime = [num2str(time(1,2)) '/' num2str(time(1,3)) '/' num2str(time(1,1))];

exportData = [cellstr(exportTime),cellstr(motorProfile),cellstr(vehicleProfile),optimum_ratio,];

[success,message] = xlsappend(filename,exportData,'Sheet1');

if success
    fprintf('append sucessful to %s \n',filename);
else
    fprintf('append unsucessful to %s \n',filename);
end