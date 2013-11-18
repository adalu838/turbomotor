%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Labskelett f�r Projekt2      %%%
%%% TSFS01 - Fordonssystem       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
load turboMap; clc;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      S�tter upp turbomotordata med antaganden     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  L�s in modellparametrar fr�n Projekt1 f�r att f� basmotorn alla parametrar
run Init_Project1.m;
% M�nga av modellparametrarna har samma v�rden som i Projekt1, exempelvis:
%
% V_em, V_im, V_es, PI_bl, r_c
%
% Dessa beh�ver vi inte �tg�rda, och har f�tt sin tilldelning i ovanst�ende
% anrop av "Init_Project1.m"

% Slagvolymen �ndras dock, varf�r n�gra  parametrar beh�ver r�knas om:
V_D_downsized = 1.2e-3; % Vi �nskar modellera en 1.2Ls motor...
n_cyl = 4;              % ...med 4 cylindrar.
V_d_downsised = V_D_downsized/n_cyl; % Cylinderslagvolym
s = (V_d_downsised*4/pi)^(1/3);      % Slagl�ngd, under antagande om kubisk motor B=s ("square bore")
B = s;                               % "Kubisk" motor ("square bore")
l = 3*s;
a = s/2;
V_d = B^2/4*pi*s; 
V_D = n_cyl*V_d; % D�p om variabeln
n_r = 2;            % Varv per cykel

% En extra kontrollvolym anv�nds mellan kompressorn och trotteln:
V_ic      = 10e-3;        % Volym fr�n intercooler, och r�rsystem mellan kompressorn och insugsr�ret
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Delmodell 1                    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load turboMap;

D = 56e-03;
N = comp.NcCorr*sqrt(comp.T01./comp.TCref);

Pi_c = comp.PiC;
T_af = comp.T01;
c_p = cp_air;
WcCorr = comp.WcCorr;
U_2 = N*(2*pi/60)*D/2;
gamma = gamma_air;

fn1 = @(a,x) a(2).*sqrt(1-(x(:,1)./(x(:,2).^2*a(1)./(2*c_p*T_af)+1).^(gamma/(gamma-1))).^2);
xdata =[Pi_c U_2];
ydata = comp.WcCorr;
comp_para1 = lsqcurvefit(fn1,[1 0], xdata, ydata);

%Haxx
Pi_c(35) = NaN;
U_2(35) = NaN;

m_dot_c_corr_max = comp_para1(2);
Psi_max = comp_para1(1);

m_c_corr_model = m_dot_c_corr_max.*sqrt(1-(Pi_c./(U_2.^2*Psi_max./(2*c_p*T_af)+1).^(gamma/(gamma-1))).^2);
m_c_corr_model(35) = NaN;
WcCorr(35) = NaN;

figure(1); clf;
plot(reshape(m_c_corr_model,7,5),reshape(Pi_c,7,5), 'r--o');
hold on;
plot(reshape(WcCorr,7,5), reshape(Pi_c,7,5), 'b-o');
ylabel('\Pi_c');
xlabel('mdot_{c,corr}')

% Compressorns effektivitet
load turboMap;
Pi_c = comp.PiC;
WcCorr = comp.WcCorr;
eta_c = comp.etaC;

% param_a = WcCorr - WcCorr_atmax;
% param_b = sqrt(Pi_c - 1) - (Pi_c_atmax - 1);
% A = [param_a.^2 2*param_a.*param_b param_b.^2 ones(34,1)];
% B = eta_c;
% Q = A\B;

fn2 = @(a,x) a(4) + a(1).*(x(:,1) - a(5)).^2 + 2*a(2).*(x(:,1) - a(5)).*(sqrt(x(:,2) - 1) + 1 - a(6)) ...
      + a(3).*(sqrt(x(:,2) - 1) + 1 - a(6)).^2;
xdata =[WcCorr Pi_c];
ydata = eta_c;
comp_para2 = lsqcurvefit(fn2,[1 1 1 0.8 0.07 1.5], xdata, ydata);

eta_c_model = comp_para2(4) + comp_para2(1).*(WcCorr - comp_para2(5)).^2 + ... 
              2*comp_para2(2).*(WcCorr - comp_para2(5)).*(sqrt(Pi_c - 1) + 1 - comp_para2(6)) ...
              + comp_para2(3).*(sqrt(Pi_c - 1) + 1 - comp_para2(6)).^2;

eta_c(35) = NaN;
WcCorr(35) = NaN;
eta_c_model(35) = NaN;
figure(2); clf;
plot(reshape(WcCorr,7,5),reshape(eta_c,7,5),'b-o');
hold on;
plot(reshape(WcCorr,7,5),reshape(eta_c_model,7,5),'r--*');

T_ref = comp.TCref;
p_ref = comp.pCref;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Delmodell 2                    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% o.s.v..

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Nedanst�ende kommer att beh�va �ndras. St�r endast med f�r att f� ett simulerbart skal!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
KpThr = 1e-6; % Throttle controller feedback setup
TiThr = 0.1;

KpWg  = 1e-6; % Wastegate controller setup
TiWg  = 4;

T_ic      = 1; % Temperature in intercooler control volume 
tau_wg    = 1; % Wastegate actuator dynamics, estimated from measurement data
dC2       = 1; % Outer compressor impeller diameter, measured by the students.
dT1       = 1; % Outer turbine impeller diameter, measured by the students.