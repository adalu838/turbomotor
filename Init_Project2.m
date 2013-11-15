%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Labskelett för Projekt2      %%%
%%% TSFS01 - Fordonssystem       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
load turbomap;
load TqEvsNeMAP;
load grupp2_wastegate;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Sätter upp turbomotordata med antaganden     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Läs in modellparametrar från Projekt1 för att få basmotorn alla parametrar
%run Init_Project1.m;
% Många av modellparametrarna har samma värden som i Projekt1, exempelvis:
%
% V_em, V_im, V_es, PI_bl, r_c
%
% Dessa behöver vi inte åtgärda, och har fått sin tilldelning i ovanstående
% anrop av "Init_Project1.m"

% Slagvolymen ändras dock, varför några  parametrar behöver räknas om:
V_D_downsized = 1.2e-3; % Vi önskar modellera en 1.2Ls motor...
n_cyl = 4;              % ...med 4 cylindrar.
V_d_downsised = V_D_downsized/n_cyl; % Cylinderslagvolym
s = (V_d_downsised*4/pi)^(1/3);      % Slaglängd, under antagande om kubisk motor B=s ("square bore")
B = s;                               % "Kubisk" motor ("square bore")
l = 3*s;
a = s/2;
V_d = B^2/4*pi*s; 
V_D = n_cyl*V_d; % Döp om variabeln
n_r = 2;            % Varv per cykel

% En extra kontrollvolym används mellan kompressorn och trotteln:
V_ic      = 10e-3;        % Volym från intercooler, och rörsystem mellan kompressorn och insugsröret
T_ic      = T_im;         % Antag isoterm modell med T_ic=T_im
J_tc      = 2.55e-5;      % Turbocharger inertia. From Westin:2002 for Mitsubishi Heavy Industry TD04HL-15T [kg m^2]
c_tc_fric = 1e-6;         % Turbo shaft friction constant
Cd_wg     = 0.9;          % Assumed value for WG discharge constant
A_max_wg  = 0.035^2/4*pi; % Measured approximation of maximum opening area of WG valve
dP_thrREF = 10e3;         % Default desired pressure loss over the throttle


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initiera I/O abstraction layer %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N_e_manual = 0; N_e_step = 1; NINI = 2000; NEND = NINI; NeST=30; NeSlope = 1; NeStartTime = 60; NeRampInit = 800;
alpha_REF_manual = 0; alphaINI = 0.0; alphaEND = alphaINI;
wg_REF_manual = 0; wgINI = 100; wgEND = wgINI; wgST=30;
pedPos_manual = 0; pedINI = 0.2; pedEND = 1.0; pedST=30;

%%%%%%%%%%%%%%%%%%%%%%
%% Compressormodell %%
%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%
%% Turbinmodell %%
%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%
%% Intercoolermodell %%
%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Nedanstående kommer att behöva ändras. Står endast med för att få ett simulerbart skal!%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
KpThr = 1e-6; % Throttle controller feedback setup
TiThr = 0.1;

KpWg  = 1e-6; % Wastegate controller setup
TiWg  = 4;

T_ic      = 1; % Temperature in intercooler control volume 
tau_wg    = 1; % Wastegate actuator dynamics, estimated from measurement data
dC2       = 1; % Outer compressor impeller diameter, measured by the students.
dT1       = 1; % Outer turbine impeller diameter, measured by the students.