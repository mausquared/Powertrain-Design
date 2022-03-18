
%% 4AUB10 - Electric and hybrid powertrain design
%  Academic year 2021-2022, Q3
%  Runner for 'EV_1spd.slx' model
clear all; close all; clc;
warning('off','all');

%% add QSS toolbox to the path
p = genpath('QSS_TB');
addpath(p);

%% Global variables
global N_sim

%% Model name
sys_name = 'EV_1spd_4AUB10';
open(sys_name);

%% Parameters definition
% Model parameters for EV with 1-spd
x = [40 40 2 40 20 22];  % set of input design parameters, x
g1        = x(1);  % gear ratio [-]
g1        = x(2);  % gear ratio [-]
scale_EM  = x(3);  % EM scale [-]
scale_EM  = x(4);  % EM scale [-]
Np        = x(5);  % Number of cells connected in parallel [-]
Ns        = x(6);  % Number of cells connected in series [-]

% Gear box 
e_gb    = 0.97;   % internal efficiency [-]
Ploss   = 100;    % stationary losses [W]
wem_min = 1;      % Minimum wheel speed beyond which losses are generated [rad/s]

% Electric machine
J_em = 0.1;       % rotating inertia [kgm2]
Paux = 0;         % auxilary power losses [W]

% Battery 
init_SoC  = 90;   % initial rel. state-of-charge [%]
E_density = 250;  % battery energy density [Wh/kg]
cell_cap  = 5.5;  % nominal cell capacity [Ah]
cell_vol  = 3.425;% nominal cell voltage [V]
set_param([sys_name, '/Battery'],'bt_Np',num2str(Np));
set_param([sys_name, '/Battery'],'bt_Ns',num2str(Ns));

% Component and vehicle mass
init_EM;                                      % init files
mem    = max(T_EM_max.*w_EM_max'/1e3)/1.4;    % E-machine mass [kg]
mb     = (Np*Ns*5.5*3.425)/E_density;         % battery mass [kg]
mtr    = 17;                                  % 1spd transmission mass
m0     = 1800;                                % curb mass [kg]
mcargo = 24000;                                   % cargo mass [kg]
mv     = m0 + mb + mem + mtr + mcargo;        % vehicle mass [kg]

% Vehicle parameters
f_r    = 0.4;       %reg. brake fraction, RWD [-]
lambda = 1.05;      % relative rotating inertia parameter [-]
cr     = 0.008;     % rolling resistance coefficient [-]
cd     = 0.23;      % air drag coefficient [-]
Af     = 2.43;      % frontal area [m2]
dw     = 0.7189;    % wheel diameter [m]

%% Performance calculations
% Acceleration
g = 9.81;        % Gravitation constant
rho = 1.18;      % Air density
kR = 0.55;       % weight distribution during acceleration
muP = 0.90;      % max. tyre-road adhesion coefficient
Ttmax = kR*muP*mv*g*dw/2;  % max. slip / tractive torque
Pemmax = max(T_EM_max.*w_EM_max');
Ptmax = Pemmax*0.97;
vb = Ptmax./(Ttmax./(dw/2)); % base vehicle speed
vf = 100/3.6;                % final speed acceleration
ta = (lambda*mv*(vb^2+vf^2))./(2*(Ptmax - (2/3)*mv*g*cr*vf - (1/5)*rho*cd*Af*vf^3));

% Max. speed vehicle without overrevving machine
vmax = (dw/2)* max(w_EM_max)/g1; % [m/s]

%% Simulation
results = sim(sys_name);

% Consider last value of the computed fuel consumption vector
cons_result = results.cons_BT(end);

% Check whether cycle could be finished exactly in N_sim computational steps;
% if cycle duration is less than N_sim, set fuel consumption to infinite
if (max(size(results.t)) < N_sim)
    cons_result = Inf;
end

%% Data display
disp(['Energy consumption  = ', num2str(cons_result),' [kWh/100km]']);
disp(['E-machine size      = ', num2str(max(T_EM_max.*w_EM_max'/1e3)), ' [kW]']);
disp(['Energy battery pack = ', num2str(Np*Ns*5.5*3.425/1e3), ' [kWh]']);
disp(['E-machine mass      = ', num2str(mem), ' [kg]']);
disp(['Battery mass        = ', num2str(mb), ' [kg]']);
disp(['Transmission mass   = ', num2str(mtr), ' [kg]']);
disp(['Vehicle mass        = ', num2str(mv), ' [kg]']);
disp(['Acceleration time   = ', num2str(ta), ' [s]']);
disp(['Max. speed          = ', num2str(vmax*3.6), ' [km/h]']);

%% Plots (Simulink model data)
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Vehicle speed, torque, power')
subplot(221), plotyy(results.t, results.w_wheel, results.t, 3.6*results.w_wheel*dw/2);
ylabel('\omega_w [rad/s], or v [km/h]'); 
grid;
subplot(222), plot(results.t, results.T_wheel); 
ylabel('T_w [Nm]'); 
grid;
subplot(223), plot(results.t, results.P_wheel/1e3); 
ylabel('P_w [kW]'); 
grid; 
xlabel('Time [s]');
subplot(224), plot(results.t, results.P_wheel/1e3, results.t, results.P_trans/1e3,...
results.t, results.P_EM/1e3, results.t, results.P_BT/1e3); grid;
xlabel('Time [s]');
ylabel('[kW]');
legend('P_{wheel}','P_{trans}', 'P_{EM}', 'P_{BT,output}');

fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Powers')
plot(results.P_wheel/1e3, results.P_trans/1e3,'.', results.P_wheel/1e3,...
results.P_EM/1e3,'o', results.P_wheel/1e3, results.P_BT/1e3,'.');
grid; 
legend('P_{trans}', 'P_{EM}', 'P_{BT,output}');
xlabel('P_{wheel} [kW]'); ylabel('Powers [kW]');

fig = figure;
set(fig,'NumberTitle', 'off') 
set(fig,'Name', 'Battery state of charge')
plot(results.t, results.SoC);
grid;
ylabel('SoC [%]');
xlabel('Time [s]');
ylim([0 100]);

fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Operation points')
v = 0:0.05:2;
[c,h] = contourf(w_EM_row*30/pi, T_EM_col, eta_EM_map',v); 
clabel(c,h); 
hold on;
plot(w_EM_max*30/pi, T_EM_max, w_EM_max*30/pi, -T_EM_max, 'r', 'linewidth', 2);
plot(results.w_EM*30/pi, results.T_EM, 'o');
xlabel('w_{EM} [rpm]'); ylabel('T_{EM} [Nm]'); 
grid;
