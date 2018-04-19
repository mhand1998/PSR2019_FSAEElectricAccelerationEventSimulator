clc
close
clear    %crtical to only get relevant variables in motor profile

%Load Motor Torque Vs Speed Curve

[TSCurveFile,PathName] = uigetfile('','Slect Motor Torque vs Speed Curve','*.csv');  
motor = csvread(fullfile(PathName,TSCurveFile));

%Enter Motor Electrical Data

currentAnswer = questdlg('Is Current vs. Torque Linear?');
switch currentAnswer
    case 'Yes'
        torque_constant = inputdlg('Enter Torque Constant[Nm/A rms]');
        torqueCurrentCurve = false;
        torque_constant = str2double(torque_constant);
    case 'No'
        %CTCurveFile = uigetfile('','Slect Motor Current vs Torque','*.csv');
        %motorCurrentCurve = csvread(TSCurveFile);
        %delimiter = 2;
        torque_constant = inputdlg('Enter Torque Constant[Nm/A rms]');
        torqueCurrentCurve  = true;
        torque_constant = str2double(torque_constant);
end

BackEMF_constant = inputdlg('Enter Back EMF Constant [RPM/V rms]');
BackEMF_constant = str2double(BackEMF_constant);
winding_resistance = inputdlg('Enter Motor Winding Resistance [Ohms]');
winding_resistance = str2double(winding_resistance);
peak_full_load_speed = inputdlg('Enter Peak Speed [RPM]');
peak_full_load_speed = str2double(peak_full_load_speed);

%Enter Motor Physical Data
motorMass = inputdlg('Enter Motor Mass [kg]');
motorMass = str2double(motorMass);

controllerMass = 9;     %controller mass [kg] (doesnt change from motor to motor)

twoMotorAnswer = questdlg('Dual Motors?');
switch twoMotorAnswer
    case 'Yes'
        motor(:,2)  = motor(:,2)*2;
        motorMass = motorMass.*2;
        controllerMass = controllerMass.*2;
        dualMotor = true;
    case 'No'
        dualMotor = false;
end

MotorProfileRoot = inputdlg('Enter File Name');
MotorProfileRoot = MotorProfileRoot{1};
MotorProfileName = strcat(MotorProfileRoot,'.mat');

save(MotorProfileName);