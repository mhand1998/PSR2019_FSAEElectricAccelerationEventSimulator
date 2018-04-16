%Load Motor Torque Vs Speed Curve

[TSCurveFile,PathName] = uigetfile('','Slect Motor Torque vs Speed Curve','*.csv');  
motor = csvread(fullfile(PathName,TSCurveFile));

%Enter Motor Electrical Data

currentAnswer = questdlg('Is Current vs. Torque Linear?');
switch currentAnswer
    case 'Yes'
        torque_constant = inputdlg('Enter Torque Constant[Nm/A rms]');
        delimiter = 1;
        torque_constant = str2double(torque_constant);
    case 'No'
        %CTCurveFile = uigetfile('','Slect Motor Current vs Torque','*.csv');
        %motorCurrentCurve = csvread(TSCurveFile);
        %delimiter = 2;
        torque_constant = inputdlg('Enter Torque Constant[Nm/A rms]');
        delimiter = 1;
        torque_constant = str2double(torque_constant);
end

BackEMF_constant = inputdlg('Enter Back EMF Constant [RPM/V rms]');
BackEMF_constant = str2double(BackEMF_constant);
winding_resistance = inputdlg('Enter Motor Winding Resistance [Ohms]');
winding_resistance = str2double(winding_resistance);
peak_full_load_speed = inputdlg('Enter Peak Speed [RPM]');
peak_full_load_speed = str2double(peak_full_load_speed);

TSCurveFileRoot = erase(TSCurveFile,'.csv');
MotorProfileName = strcat(TSCurveFileRoot,'.mat');

save(MotorProfileName);