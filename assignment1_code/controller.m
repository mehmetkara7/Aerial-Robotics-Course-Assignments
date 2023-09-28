function [u] = pd_controller(~, s, s_des, params)
% Simple PD Controller
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% PD controller gains
Kp = 100;
Kv = 16;

% Calculate error terms
e = s_des(1) - s(1);
edot = s_des(2) - s(2);

% Calculate PD control signal
u = params.mass * (Kp * e + Kv * edot + params.gravity);

% Limit the control signal
u = max(min(u, params.max_force), -params.max_force);

end
