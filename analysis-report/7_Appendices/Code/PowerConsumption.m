%% Variables
clc; clear;
m1 = 0.836;
m2 = 0.532;
N = 160;
f = 10;
r1 = 100;
r2 = 50;
r3 = 300;
ed = 50;
dd =0;
ddd=0;
edd=0;
alpha = 68.58;
length_begin = 124;
length_end = 241; % taken from get_max_extension.m
precision = 0.001;
e = length_begin : precision : length_end;
array_size = (length_end-length_begin)/precision +1;
d = 210;
gearRatio=100;

Torque1 = zeros(3,array_size);
Torque2 = zeros(3,array_size);
Torque1_spring = zeros(3,array_size);
Torque2_spring = zeros(3,array_size);
spring1 = zeros(3,array_size);
spring2 = zeros(3,array_size);
theta = zeros(array_size,1);
thetad = zeros(array_size,1);
thetadd =zeros(array_size,1);
phi = zeros(array_size,1);
phid = zeros(array_size,1);
phidd = zeros(array_size,1);
psy = zeros(array_size,1);

%% Get Torque Phase 1

for i = 1:1:array_size 
    [theta(i), phi(i), psy(i), R] = leg_angles(r1,r2,r3,alpha,d,e(i));
    [thetad(i), phid(i)] = leg_angles_dot(r1,R,theta(i),phi(i),dd,ed);
    [thetadd(i), phidd(i)] = leg_angular_acceleration(r1,R,theta(i),thetad(i),phi(i),phid(i),ddd,edd);
    [Torque1(1,i),Torque2(1,i)] = Dynamic_Equation(m1,m2,R/1000,r1/1000, theta(i), phi(i),thetadd(i), phidd(i), N,f);
    [Torque1_spring(1,i),Torque2_spring(1,i)] = Dynamic_Equation_Springs(m1,m2,R/1000,r1/1000, theta(i), phi(i),thetadd(i), phidd(i), N,f);
    [spring1(1,i),spring2(1,i)] = Spring(theta(i), phi(i));
end

%% Get Torque Phase 2 - Phi increases

for i = 1:1:array_size 
    [Torque1(2,i),Torque2(2,i)] = Dynamic_Equation(m1,m2,R/1000,r1/1000, theta(1), phi(i),0, 0, 0,0);
    [Torque1_spring(2,i),Torque2_spring(2,i)] = Dynamic_Equation_Springs(m1,m2,R/1000,r1/1000, theta(1), phi(i),0, 0, 0,0);
    [spring1(2,i),spring2(2,i)] = Spring(theta(1), phi(i));
end

%% Get Torque Phase 3 - Theta Decreases

for i = 1:1:array_size 
    [Torque1(3,i),Torque2(3,i)] = Dynamic_Equation(m1,m2,R/1000,r1/1000, theta(i), phi(array_size),0, 0, 0,0);
    [Torque1_spring(3,i),Torque2_spring(3,i)] = Dynamic_Equation_Springs(m1,m2,R/1000,r1/1000, theta(i), phi(array_size),0, 0, 0,0);
    [spring1(3,i),spring2(3,i)] = Spring(theta(i), phi(array_size));
end

%% Get Harmonic Drive and Motor Specs

Max_Torque1 = max([max(abs(Torque1(1,:))),max(abs(Torque1(2,:))),max(abs(Torque1(3,:)))]);
Max_Torque2 = max([max(abs(Torque2(1,:))),max(abs(Torque2(2,:))),max(abs(Torque2(3,:)))]);
Max_Torque1_spring = max([max(abs(Torque1_spring(1,:))),max(abs(Torque1_spring(2,:))), max(abs(Torque1_spring(3,:)))]);
Max_Torque2_spring = max([max(abs(Torque2_spring(1,:))),max(abs(Torque2_spring(2,:))),max(abs(Torque2_spring(3,:)))]);
Min_Torque1_spring = min([min(abs(Torque1_spring(1,:))),min(abs(Torque1_spring(2,:))), min(abs(Torque1_spring(3,:)))]);
Min_Torque2_spring = min([min(abs(Torque2_spring(1,:))),min(abs(Torque2_spring(2,:))),min(abs(Torque2_spring(3,:)))]);
Min_thetad = min(abs(thetad));
Min_phid = min(abs(phid));

HdSpecs_theta = get_hd_specs(Max_Torque1_spring);
HdSpecs_phi = get_hd_specs(Max_Torque2_spring);

% get_hd_inputs() gives results in mNm and rpm
[Max_Torque1_motor,Min_thetad_motor] = get_hd_inputs(Min_thetad,Max_Torque1_spring,HdSpecs_theta(3),gearRatio);
[Max_Torque2_motor,Min_phid_motor] = get_hd_inputs(Min_phid,Max_Torque2_spring,HdSpecs_phi(3),gearRatio);
MotorSpecs_theta = get_motor_specs(Max_Torque1_motor); % get_motor_specs takes input in mNm
MotorSpecs_phi = get_motor_specs(Max_Torque2_motor);

%% Phase Time
Phase_Time = [(length_end-length_begin)/ed,(max(phi)-min(phi))/(mean(abs(phid))*180/pi),(max(theta)-min(theta))/(mean(abs(thetad))*180/pi)];
Step_Time = Phase_Time./array_size;


%% Joules Consumed
Joules_Consumed = [0,0,0];
Coulombs_Consumed = [0,0,0];
thetad_mean = mean(abs(thetad));
phid_mean = mean(abs(phid));

for i = 1:1:array_size
    %% Get Joules Phase 1
    [torque_motor_theta,rpm_motor_theta] = get_hd_inputs(abs(thetad(i)),abs(Torque1_spring(1,i)),HdSpecs_theta(3),gearRatio);
    [torque_motor_phi,rpm_motor_phi] = get_hd_inputs(abs(phid(i)),abs(Torque2_spring(1,i)),HdSpecs_phi(3),gearRatio);
    [joulestheta,coulombstheta] = get_motor_inputs(torque_motor_theta,rpm_motor_theta,MotorSpecs_theta(6),MotorSpecs_theta(7),MotorSpecs_theta(5));
    [joulesphi,coulombsphi] = get_motor_inputs(torque_motor_phi,rpm_motor_phi,MotorSpecs_phi(6),MotorSpecs_phi(7),MotorSpecs_phi(5));
    Coulombs_Consumed(1) = Coulombs_Consumed(1) + (coulombstheta+coulombsphi)*Step_Time(1);
    Joules_Consumed(1) = Joules_Consumed(1) + (joulestheta+joulesphi)*Step_Time(1);

    %% Get Joules Phase 2
    [torque_motor_theta,rpm_motor_theta] = get_hd_inputs(0,abs(Torque1_spring(2,i)),HdSpecs_theta(3),gearRatio);
    [torque_motor_phi,rpm_motor_phi] = get_hd_inputs(abs(phid_mean),abs(Torque2_spring(2,i)),HdSpecs_phi(3),gearRatio);
    [joulestheta,coulombstheta] = get_motor_inputs(torque_motor_theta,rpm_motor_theta,MotorSpecs_theta(6),MotorSpecs_theta(7),MotorSpecs_theta(5));
    [joulesphi,coulombsphi] = get_motor_inputs(torque_motor_phi,rpm_motor_phi,MotorSpecs_phi(6),MotorSpecs_phi(7),MotorSpecs_phi(5));
    Coulombs_Consumed(2) = Coulombs_Consumed(2) + (coulombstheta+coulombsphi)*Step_Time(2);
    Joules_Consumed(2) = Joules_Consumed(2) + (joulestheta+joulesphi)*Step_Time(2);

    %% Get Joules Phase 3
    [torque_motor_theta,rpm_motor_theta] = get_hd_inputs(abs(thetad_mean),abs(Torque1_spring(3,i)),HdSpecs_theta(3),gearRatio);
    [torque_motor_phi,rpm_motor_phi] = get_hd_inputs(0,abs(Torque2_spring(3,i)),HdSpecs_phi(3),gearRatio);
    [joulestheta,coulombstheta] = get_motor_inputs(torque_motor_theta,rpm_motor_theta,MotorSpecs_theta(6),MotorSpecs_theta(7),MotorSpecs_theta(5));
    [joulesphi,coulombsphi] = get_motor_inputs(torque_motor_phi,rpm_motor_phi,MotorSpecs_phi(6),MotorSpecs_phi(7),MotorSpecs_phi(5));
    Coulombs_Consumed(3) = Coulombs_Consumed(3) + (coulombstheta+coulombsphi)*Step_Time(3);
    Joules_Consumed(3) = Joules_Consumed(3) + (joulestheta+joulesphi)*Step_Time(3);
end

%% Amps and Watts
Amps_phase = [Coulombs_Consumed(1)/Phase_Time(1),Coulombs_Consumed(2)/Phase_Time(2),Coulombs_Consumed(3)/Phase_Time(3)];
Amps_Consumed = (sum(Coulombs_Consumed)*5)/(Phase_Time(1) + Phase_Time(2)*5 + Phase_Time(3)*5);
Watts_Consumed_Phase = [Joules_Consumed(1)/Phase_Time(1),Joules_Consumed(2)/Phase_Time(2),Joules_Consumed(3)/Phase_Time(3)];
Watts_Consumed = (sum(Joules_Consumed)*5)/(Phase_Time(1) + Phase_Time(2)*5+Phase_Time(3)*5);
fprintf('Average Current (no other electronics): %fA\n',Amps_Consumed);

current_other_components = 2.133;
Amps_Consumed = Amps_Consumed + current_other_components;
fprintf('Average Current (with other electronics): %fA\n',Amps_Consumed);

ah_per_cell = 3.4;
v_per_cell = 3.6;
kg_per_cell = 0.05;
hours = 2;
voltage = 48;
cells_series = ceil(voltage/v_per_cell);
cells_parallel = ceil(Amps_Consumed*hours/ah_per_cell);
cells_total = cells_parallel*cells_series;
kg_total = cells_total*kg_per_cell;

fprintf('Number of Cells: %f\n',cells_total);
fprintf('Weight of battery: %fkg\n',kg_total);