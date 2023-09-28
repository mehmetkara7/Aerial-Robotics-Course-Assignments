function [u1, u2] = controller(~, state, des_state, params)
% Simple Planar Quadrotor Controller
%
%   state: Current robot state with fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: Desired states:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: Robot parameters

% Robot data
m = params.mass;
Ixx = params.Ixx;
g = params.gravity;

% Control gains
Kvz = 10;
Kpz = 50;
Kvphi = 20;
Kpphi = 80;
Kvy = 10;
Kpy = 50;

% Current state
y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);
phi = state.rot;
phi_dot = state.omega;

% Desired state
y_des = des_state.pos(1);
z_des = des_state.pos(2);
y_dot_des = des_state.vel(1);
z_dot_des = des_state.vel(2);
y_ddot_des = des_state.acc(1);
z_ddot_des = des_state.acc(2);

% Control equations
phi_c = -(y_ddot_des + Kvz * (z_dot_des - z_dot) + Kpz * (z_des - z)) / g;
phi_c_dot = -(Kvy * (y_ddot_des + g * phi) + Kpy * (y_dot_des - y_dot)) / g;
u1 = m * (g + z_ddot_des + Kvz * (z_dot_des - z_dot) + Kpz * (z_des - z));
u2 = Ixx * (Kvphi * (phi_c_dot - phi_dot) + Kpphi * (phi_c - phi));

end
