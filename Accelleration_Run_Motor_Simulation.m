%{
Electric Car Accelleration Event Simulation
Author:Michael Hand
Started: 3/5/17
This file is subject to the terms and conditions defined in file 'LICENSE.txt', which is part of this source code package.
%}


%%
clc
close

%Load Vehicle Profile

[vehicleProfile,PathName] = uigetfile('','Load Vehcile Profile','*.mat');
load(fullfile(PathName,vehicleProfile))

%%

%Load Motor Torque Vs Speed Curve

[TSCurveFile,PathName] = uigetfile('','Slect Motor Torque vs Speed Curve','*.csv');  
motor = csvread(fullfile(PathName,TSCurveFile));

%%

%Enter Motor Electrical Data

currentAnswer = questdlg('Is Current vs. Torque Linear?');
switch currentAnswer
    case 'Yes'
        torque_constant = inputdlg('Enter Torque Constant[Nm/A rms]');
        currentCurve = false;
        torque_constant = str2double(torque_constant);
    case 'No'
        [CTCurveFile,PathName] = uigetfile('','Slect Motor Current vs Torque','*.csv');
        motorCurrentCurve = csvread(fullfile(PathName,CTCurveFile));
        currentCurve = true;
end

BackEMF_constant = inputdlg('Enter Back EMF Constant [RPM/V rms]');
BackEMF_constant = str2double(BackEMF_constant);
winding_resistance = inputdlg('Enter Motor Winding Resistance [Ohms]');
winding_resistance = str2double(winding_resistance);
peak_full_load_speed = inputdlg('Enter Peak Speed [RPM]');
peak_full_load_speed = str2double(peak_full_load_speed);


%%

gear_ratio = 3.80; %[] parameter of powertrain

drag_strip_length = 75; %[m} distance for acell run


%Setup
power_limited = false;
wcir = wdia*pi;
s = zeros(1,2);     %[m] position
v = 0;              %[m/s] velocity
t_increment = .001; %[sec] dt for simulation loop
i = 1;              %[] counter

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

%If Current Curve Entered, Use Current Curve
if currentCurve
    current_idx = Closest(motorCurrentCurve(:,4),motor_torque(1,i));
    current(1,i) = motorCurrentCurve(current_idx,4);
else
    current(1,i) = motor_torque(1,i)/torque_constant;
end
    

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

%Make arrays equal length for plots
i = i-1;
a(1,(i+1:i+2)) = a(1,i);
t(1,(i+1:i+2)) = t(1,i);
v(1,(i+1:i+2)) = v(1,(i+1));
f_wheel_actual(1,(i+1:i+2)) = f_wheel_actual(1,i);
f_wheel_max(1,(i+1:i+2)) = f_wheel_max(1,i);
f_motor(1,(i+1:i+2)) = f_motor(1,i);
power(1,(i+1:i+2)) = power(1,i);
battery_draw_power(1,(i+1:i+2)) = battery_draw_power(1,i);

if power_limited
    disp('Warning: Power Limited')
end

max_current = max(current);

fprintf('Accel Time: %.3f seconds\n' ,t(1,(i-1)));

fprintf('Max Current Draw: %.3f\n', max_current)

title(TSCurveFile)
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
hold on
%plot(t,f_wheel_actual)
plot(t,f_motor,'b')
plot(t,f_wheel_max,'r')
hold off
legend ('tractive motor force','max tractive force','location','best' )
xlabel('Time (sec)')
ylabel('Wheel Force (N)')
title('Wheel Force vs. Time')
subplot(2,2,4)
plot(t,battery_draw_power)
ylim([0,100000])
xlabel('Time (sec)')
ylabel('Accumulator Power(W)')

%%
i = i + 1;

filename = 'MotorSimulationData.xls';
xlswrite(filename,cellstr(TSCurveFile),'A1:A1')
xlswrite(filename,gear_ratio,'B1:B1')
xlswrite(filename,t(1,(i-1)),'C1:C1')

