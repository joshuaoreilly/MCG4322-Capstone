function [torque_motor,rpm_motor] = get_hd_inputs(speed_output,torque_output,torque_rated,gear_ratio)
	%% INPUTS
	% speed at output of harmonic drive (rad/s)
	% torque at output of harmonic drive (Nm)
	% rated torque of harmonic drive (Nm)
	% gear ratio of harmonic drive (100:1)
	%% OUTPUTS
	% torque at output of motor/input of harmonic drive(mNm)
	% speed at output of motor/input of harmonic drive (rpm)
	
	rpm_output = speed_output*30/pi;
	rpm_motor = rpm_output*gear_ratio;
	
	%% SPEED EFFICIENCY
	eta_r = (4.848*(10^(-9)))*(rpm_motor^2) + (-5.879*(10^(-5)))*(rpm_motor) + 0.8367;
    if eta_r > 0.81
        eta_r = 0.81;
    elseif eta_r < 0.69
        eta_r = 0.69;
    end
	
	%% TORQUE EFFICIENCY
	alpha = torque_output/torque_rated;
    if alpha > 1
        alpha = 1;
    end
    k_e = (-1.481*(alpha^4))+(4.312*(alpha^3))-(5.013*(alpha^2))+(3.159*alpha)-0.02076;
    if k_e < 0.3
        k_e = 0.3;
    end
	
	%% MOTOR TORQUE
	eta_HD = eta_r*k_e;
	torque_motor = torque_output*1000/(gear_ratio*eta_HD);	
end