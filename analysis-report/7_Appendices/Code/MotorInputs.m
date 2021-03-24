function [power_motor_in,current_motor] = get_motor_inputs(torque_motor,rpm_motor,torque_constant,speed_constant,resistance_motor)
    %% INPUTS:
    % speed at output of motor (rpm)
    % torque at output of motor (mNm)
    % torque constant of motor (mNm/A)
	% speed constant of motor (rpm/V)
    % resistance of the motor (Ohms)
    %% OUTPUTS:
    % electrical power into motor (W)
    % current into motor (A)
    
	U = (1/speed_constant)*(((30000*resistance_motor*torque_motor)/(pi*torque_constant^2))+rpm_motor);
	a = resistance_motor;
	b = -U;
	c = (pi*rpm_motor*torque_motor)/(30000);
	I_upper = (-b+sqrt((b^2)-(4*a*c)))/(2*a);
	I_lower = (-b-sqrt((b^2)-(4*a*c)))/(2*a);
	% When speed is 0, I_lower = 0
	if I_lower <= 0
		current_motor = I_upper;
	else
		current_motor = I_lower;
	end
	power_motor_in = U * current_motor;
	
end