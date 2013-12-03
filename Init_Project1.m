%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         Projekt 1c TSFS09          %%%
%%%         ramse879, adalu838         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Sätter upp motordata      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

l     = 0.153;        % Vevstakens längd [m]
a     = 0.045;	      % Halva slaglängden [m]
B     = 0.090;	      % Cylinderdiameter [m]
n_cyl = 4;            % Antalet motorcylindrar
V_d   = 2*a*pi*B^2/4; % Volym i [m^3]
n_r   = 2;            % Varv per cykel
V_im  = 2e-3;         % Volym hos insugsröret.
V_em  = 2e-3;         % Avgasgrenrörsvolym
V_es  = 10e-3;        % Avgassystemsvolym
V_D   = 2.290 *10^-3; % Motorvolym
r_c   = 9.3;          % Kompressionsförhållande
PI_bl = 1.85;         % Boost layout compensator

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     S�tter upp fordonsdata     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VehicleMass = 1520; % kg
WheelRadius = 0.3;  % m
VehicleArea = 2;    % m^2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Sätter övrigt data         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gas properties
R_exh     = 280;
R_air     = 280;
gamma_air = 1.3;
gamma_exh = 1.4;
cv_air = R_air/(gamma_air-1); % J/kg/K, @T=[-50..40]C
cv_exh = R_exh/(gamma_exh-1); % J/kg/K
cp_air = gamma_air*cv_air;    % J/kg/K, @T=[-50..40]C
cp_exh = gamma_exh*cv_exh;    % J/kg/K

% Ambient pressure
p_amb=101.3e3; %kPa

% Ambient temperature
T_amb=293; %K

% Fuel properties
q_HV = 44e6;
afs  = 15.0;  % Stökiometriskt A/F (isooktan)

% Lambda sensor dynamics
tau_lambda = 0.050; % Response time of lambda sensor, 50ms

% Turbocharger inertia
J_tc  = 2.55e-5; % From Westin 2002 for MHI TD04HL-15T [kg m^2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Sätter upp förarmodell    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

KpDriver = 0.04;
KiDriver = 0.02;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Lös in motordata från Projekt 1a  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lägg till sökvägen till mätfilerna med File - Set Path
load('EngineMapTSFS09.mat');
load('grupp1_airstep.mat');
load('grupp1_fuelstep.mat');
load('lightOff.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Anpassning av trotteldynamik %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Alpha_th = grupp1_airstep.alpha(2:1001);
Beta_ped = grupp1_airstep.alpha_ref(2:1001);
t = grupp1_airstep.t(2:1001);
T = t(1);
Tau_th = T*3;

% figure(1);
% hold on;
% plot(t,Alpha_th);
% plot(t,Beta_ped,'r');
%  
% legend('\alpha_{th}','\beta_{ped}');
% xlabel('Tid (s)');
% ylabel('Vinkel (Grader)');

% Okulärundersökning och medelvärdesbildning gav

Alpha_ref = (pi/180)*grupp1_airstep.alpha(1:1001);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Anpassning av modell för effektiv area förbi trotteln %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_im = EngineMap.p_im;
p_befthr = EngineMap.p_bef_thr;
T_befthr = mean(EngineMap.T_bef_thr);
stora_pi = p_im./p_befthr;
a_thr = EngineMap.alpha;
m_dot_at = EngineMap.m_dot_at;

stora_pi_temp = stora_pi;
stora_pi_lim = max(stora_pi, ones(length(stora_pi),1)*(2/(gamma_air + 1))^(gamma_air/(gamma_air-1)));
Psi = sqrt(2*gamma_air/(gamma_air - 1)*(stora_pi_lim.^(2/gamma_air) - stora_pi_lim.^((gamma_air + 1)/gamma_air)));
a_thr_sqr = a_thr.^2;

% Minsta kvadratanpassning
A = [ones(length(stora_pi),1) a_thr a_thr_sqr];
B = m_dot_at.*sqrt(R_air*T_befthr)./(p_befthr.*Psi);

remove = [];
for i=1:length(stora_pi)
    if stora_pi(i) >= 0.8
        remove = [remove,i];
    end
end

A(remove,:) = [];
B(remove) = [];
a_thr(remove) = [];

X = A\B;
a_1 = X(1);
a_2 = X(2);
a_3 = X(3);

alfa_model = 0:1/length(stora_pi):1-1/length(stora_pi);
A_eff_model = ([ones(length(stora_pi),1) alfa_model' (alfa_model').^2]*X);

A_eff = B;

model_error = abs(A_eff - A*X)./A_eff;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Anpassning av modell för insugsrör %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = EngineMap.N;
T_im = EngineMap.T_im;
m_dot_ac = EngineMap.m_dot_at;
p_im = EngineMap.p_im;

C = [ones(344,1) sqrt(p_im) sqrt(N)];
D = (n_r*R_air)/(V_d*n_cyl)*(m_dot_ac.*T_im)./(p_im.*N);

n_vol_parameters = C\D;
n_vol = ones(length(p_im), length(N));

n_vol_model = C*n_vol_parameters;

n_vol_measured = (n_r*R_air)/(V_d*n_cyl).*(m_dot_ac.*T_im)./(p_im.*N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Anpassning av modell för bränsleinsprutningen  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = EngineMap.N;
t_inj = EngineMap.t_inj;
AFS = EngineMap.airfuel.AFs;
lambda = EngineMap.lambda_bc_cont;
mfidot = EngineMap.m_dot_at./(AFS.*lambda);
mfi = (n_r/n_cyl)*mfidot./N;

A = [t_inj (-1)*ones(length(t_inj),1)];
B = (n_r/n_cyl).*mfidot./N; 
sol = A\B;
t_0 = sol(2)/sol(1);
c = sol(1)*(n_cyl/n_r);
k_inj = sol(1);

mfimodel = k_inj*(t_inj - t_0); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Anpassning av modell för cylindern  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

M_measured = EngineMap.M_b;
N = EngineMap.N;
AFS = EngineMap.airfuel.AFs;
lambda = EngineMap.lambda_bc_cont;
mfidot = EngineMap.m_dot_at./(AFS.*lambda);
mfi = n_r*mfidot./N;
p_im = EngineMap.p_im;
p_em = EngineMap.p_em;

A = zeros(344,4);
A(:,1) = mfi*q_HV*(1- 1/r_c^(gamma_air - 1)).*min(1,lambda);
A(:,2) = -V_D*ones(344,1);
A(:,3) = -V_D*N;
A(:,4) = -V_D*N.^2;
A = A/(2*pi*n_r);

B = M_measured + V_D/(2*pi*n_r)*(p_em - p_im);

cyl_sol = A\B;

Wig = mfi*q_HV*(1- 1/r_c^(gamma_air - 1)).*min(1,lambda)*cyl_sol(1);
Wpump = V_D*(p_em - p_im);
Wfric = (cyl_sol(2) + cyl_sol(3)*60*N/1000 + cyl_sol(4)*(60*N/1000).^2)*V_D;
M_model = (Wig - Wpump - Wfric)/(2*pi*n_r);

M_error = abs(M_measured - M_model)/M_measured;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Anpassning av modell för avgasrör   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_em = EngineMap.T_t;
AFS = EngineMap.airfuel.AFs;
lambda = EngineMap.lambda_bc_cont;
macdot = EngineMap.m_dot_at;
mfidot = macdot./(AFS.*lambda);
p_em = EngineMap.p_t;

C_mat = p_em.*max(0,p_em - p_amb*ones(344,1))./((mfidot + macdot).^2.*R_exh.*T_em);
C_exh = mean(C_mat);

p_model = p_amb/2 + sqrt(ones(344,1)*p_amb^2/4 + C_exh*(mfidot + macdot).^2*R_exh.*T_em);
p_error = abs(p_em - p_model)/p_em;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Anpassning av modell för lambdagivare   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_dot_air = grupp1_fuelstep.m_dot_air'*0.3462*10^-3;
lambda_c = grupp1_fuelstep.lambda_bc_cont';
t_inj = grupp1_fuelstep.t_inj';
t = grupp1_fuelstep.t';
T = t(2);
N = 25;
mfidot = N*c.*(t_inj - ones(size(t_inj))*0);
mfidot_test = m_dot_air./lambda_c/AFS;

tau_d =0.38;
tau_mix = 0.1;
T_befthr = mean(T_befthr);
T_em = mean(T_em);
T_im = mean(T_im);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Beräknar rull- och  luftmotstånd %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F = [0 410 1050]'; %N
v = [0 80 160]';   %km/h
rho_air=p_amb/(R_air*T_amb);
A=[VehicleMass*ones(size(v)) VehicleMass*v/3.6 0.5*VehicleArea*rho_air*(v/3.6).^2];
x=A\F;
c_r1=x(1);
c_r2=x(2);
C_d=x(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Löser in data för körcykeln   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init_drivecycle;

% Inlösta data för europeiska körcykeln:
% DrivingScenario : Matris ur vilken nedanstående information extraherats
% S : Referenshastighet [km/h]
% U : Kopplingens läge 
% G : Vald växel [0-5]
% T : Tidsvektor [s]                  

% Bildar vektorer till f�rarmodellen
Clutch = [T U];
Gear   = [T max(G,1)];
Speed  = [T S];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------------------------------- %
% Efterbehandling när simuleringen är klar %
% ---------------------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Light-Off beräkning       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot(lightOff.time,lightOff.lambda_bc_disc,'-b',lightOff.time,lightOff.lambda_ac_disc, 'r');
% xlabel('Tid (s)');
% ylabel('Lambda');

light_Off_time = 37.5; %sekunder

% 
% %%%%%%%%%%%%%%%%%%%%%%
% %%    Pedalsteg    %%%
% %%%%%%%%%%%%%%%%%%%%%%
% 
% plot(tourque_step)
% ylabel('Vridmoment (Nm)')
% xlabel('Tid (Sekunder)')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%    Full gas p� 4:e v�xeln     %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% plot(VehicleSpeed)
% grid on;
% xlabel('Tid (S)')
% ylabel('Hastighet (Km/h)')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%    K�rning p� de 30 f�rsta sekunderna    %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure(14); clf;
% plot(VehicleSpeed)
% xlabel('Tid (S)')
% ylabel('Hastighet (Km/h)')
% 
% figure(15); clf;
% plot(t_sim, p_im_cyk)
% xlabel('Tid (S)')
% ylabel('Insugstryck (Pa)')
% 
% figure(16); clf;
% plot(t_sim, Tq_e)
% xlabel('Tid (S)')
% ylabel('Vridmoment (Nm)')
% 
% figure(17); clf;
% plot(t_sim, lambda_cyl)
% xlabel('Tid (S)')
% ylabel('Lambda i cylindern')
% 
% K_i = 0.02;
% K_p = 0.009; %nvol = 0.65121
% 
% %$K_p = 0.0205$ och $K_i = 0.082$. //Gammal
% %plot(t_sim, Lambda_cont);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Beräkna emissioner      %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Argument till calcEmissions  %%%
% tout      : Tidsvektor från simulink
tout = t_sim;
%  lambda    : Kontinerligt lambda
lambda = Lambda_cont;
%  Distance  : Körd sträcka i meter
Distance = VehicleDistance;
%  dmacAct   : Luftmassflöde till cylindern i kg/s
dmacAct = EngineAir;
%  dmfAct    : Bränsleflöde till cylinder i kg/s
dmfcAct = EngineFuel;
%  lightOff  : Tid i sekunder tills light-Off
lightOff = 37.5;

calcemissions(tout, lambda, Distance, dmacAct, dmfcAct, lightOff);

% Ber�kna bränsleförbrukningen
fuelCons = EngineFuelAcc(length(EngineFuelAcc))/Distance(length(Distance))*100000;
  
disp(sprintf('Br�nslef�rbrukning: %1.2f [l/(10 mil)]',fuelCons));