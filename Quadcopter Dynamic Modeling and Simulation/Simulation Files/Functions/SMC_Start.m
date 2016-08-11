%% Nonlinear Control of Quadrotor
%% SMC Initial Setup
% Retrieve Parameter from Model
global Model
% Model = load('Interboard-Multiwii.mat');
% load('Hover.mat')
Ix = quadModel.Jx;
Iy = quadModel.Jy;
Iz = quadModel.Jz;
Jr = 3.441e-04; % Propeller Inertia Using 2/3*MR^2 [kg*m^2]
l = quadModel.arms_L; % Quadrotor CoG to Motor Length

a1 = (Iy-Iz)/Ix;
a2 = Jr/Ix;
a3 = (Iz-Ix)/Iy;
a4 = Jr/Iy;
a5 = (Ix-Iy)/Iz;


b1 = l/Ix;
b2 = l/Iy;
b3 = 1/Iz;

