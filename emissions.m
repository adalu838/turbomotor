function r = emissions(lambda)
% EMISSION, emission calculation as a function of lambda.
%  result = emission(lambda);  
%    where 0.7 <= lambda <= 1.4
%
%  Result is a structure with following fieles
%
%  emissionName  This contains a list of the order of the next field, emission.
%  emH2          Emission in mole fractions of H2
%  catEffH2      Contains a value between 0.0 and 1.0 describing the
%                efficiency of the catalythic coverter for H2.
%                Is in this version always 0.
%  emCO          Emission in volume percent
%  catEffCO      Contains a value between 0.0 and 1.0 describing the
%                efficiency of the catalythic coverter for CO.
%  emNOx         Emission in volume promille
%  catEffNOx     Contains a value between 0.0 and 1.0 describing the
%                efficiency of the catalythic coverter for NOx.
%  emHC          Emission in volume promille
%  catEffHC      Contains a value between 0.0 and 1.0 describing the
%                efficiency of the catalythic coverter for HC.
%  emO2          Emission in mole fractions of O2
%  catEffO2      Contains a value between 0.0 and 1.0 describing the
%                efficiency of the catalythic coverter for O2.
%                Is in this version always 0.
%

% Av: Per Andersson
% $Revision: 1.1.1.1 $

% Set cat eff window. offset = 0.01 ==> 0.99 <=window <= 1.01
offset = 0.02;

% Remove error prone data
lambda = max(0.7+offset,lambda);
lambda = min(1.4-offset,lambda);


% Set emission names
r.emissionNames = {'H2', 'CO', 'NOx', 'HC', 'O2'};

% Initiate result to zero.

% Set up local variables
H2  = 1; % Index in matrix for H2
CO  = 2; % Index in matrix for CO
NOx = 3; % Index in matrix for NOx
HC  = 4; % Index in matrix for HC
O2  = 5; % Index in matrix for O2


% Locals for indexing in structure emission.
beforeCat   = 1; % First row
utilization = 2; % Second row

% Set up tables for interpolation
%   First column : lambda
%   2nd   column : Value in percent

% Tables for gas AfterCat is not currently used. They was used to
% calculate cat efficiency. But it did not give a satisfieying result.

H2TableBeforeCat = [
  0.67, 6.2e-2;
  0.77, 3.3e-2;
  0.83, 0.02;
  0.91, 8.8e-3;
  1.0 , 2.2e-3;
  1.11, 0;
  1.25, 0;
  1.4 , 0];

COTableBeforeCat = [
  0.7   16;
% 0.88, 7;
  0.9 , 6;
  0.96, 3;
  1.0 , 1.59;
  1.1 , 0.47;
  1.15, 0.45;
  1.2 , 0.47;
  1.3 , 0.50;
  1.4 , 0.48];

COTableAfterCat = [
  0.88 , 6;
  0.9  , 4.78;
  0.976, 0.5;
  0.99 , 0.375;
  1.004, 0.33;
  1.15 , 0.28];

COCatEff = [
  0.7 - offset,  0.0;
  0.89 - offset, 0.0;
  0.92 - offset, 0.03;
  0.95 - offset, 0.08;
  0.97 - offset, 0.123;
  0.98 - offset, 0.36;
  0.99 - offset, 0.62;
  1.0 - offset , 0.92;
  1.003 - offset, 0.985;
  1.005 - offset, 0.995;
  1.01 - offset, 1.0;
  1.025 - offset,1.0;
  1.05 - offset, 1.0;
  1.1 - offset , 1.0;
  1.15 - offset, 1.0;
  1.4 - offset , 1.0];

NOxTableBeforeCat = [
  0.7 , 0.5;
% 0.87, 1.5;
  1.0 , 2.8;
  1.03, 2.9;
  1.1 , 2.75;
  1.4 , 0.94;
  ];

NOxTableAfterCat = [
  0.9  , 0.047;
  0.99 , 0.047;
  1.0  , 0.094;
  1.004, 0.19;
  1.018, 5;
  1.032, 2.85;
  1.05 , 2.85;
  1.1  , 2.5];

NOxCatEff = [
  0.7 + offset,   1.0;
  0.8 + offset,   1.0;
  0.9 + offset,   1.0;
  0.97 + offset,  1.0;
  0.98 + offset,  1.0;
  0.99 + offset,  1.0;  
  1.0 + offset , 0.95;
  1.01 + offset, 0.2;
  1.015 + offset, 0.8*mean([0.2 0.043]);
  1.02 + offset, 0.043;
  1.03 + offset, 0.029;
  1.08 + offset, 0.02;
  1.15 + offset, 0.0;
  1.2 + offset , 0.0;
  1.4 + offset , 0.0];

HCTableBeforeCat = [
  0.7 , 5;
% 0.88, 3.5 ;
  0.9 , 3.18;
  1.0 , 2.06;
  1.1 , 1.17;
  1.14, 1;
  1.2 , 1.17;
  1.3 , 2.2;
  1.35, 3.18;
  1.4 , 5.4];

HCTableAfterCat = [
  0.9 , 0.56;
  1.0 , 0.05;
  1.1 , 0.08
  1.2 , 0.08];

HCCatEff = [
  0.7 - offset, 0.05;
% 0.88 - offset, 0.17;
  0.9 - offset , 0.18
  0.97 - offset, 0.58;
  0.98 - offset, 0.72;
  0.99 - offset, 0.84;
  1.00 - offset, 0.90;
  1.01 - offset, 0.93;
  1.02 - offset, 0.93;
  1.03 - offset, 0.93;
  1.15 - offset, 0.93;
  1.4 - offset , 0.93];

O2TableBeforeCat = [
  0.7 , 2.1e-3;
  0.77, 2.2e-3;
  0.83, 2.67e-3;
  0.91, 4.4e-3;
  1.0 , 6.67e-3;
  1.11, 2.67e-2;
  1.25, 4.8e-2;
  1.4 , 7e-2];





% Check limits of lambda:
lambda(find(lambda < 0.7)) = 0.7;
lambda(find(lambda > 1.4)) = 1.4;

% Assign to structure
    
r.emH2      = interp1(H2TableBeforeCat(:,1), H2TableBeforeCat(:,2), lambda);
r.catEffH2  = 0;

r.emCO       = spline(COTableBeforeCat(:,1), ...
    COTableBeforeCat(:,2),lambda);
r.catEffCO  = interp1(COCatEff(:,1), COCatEff(:,2) , lambda);

r.emNOx     = spline(NOxTableBeforeCat(:,1), ...
    NOxTableBeforeCat(:,2), lambda);
r.catEffNOx = interp1(NOxCatEff(:,1), NOxCatEff(:,2), lambda);

r.emHC      = spline(HCTableBeforeCat(:,1), HCTableBeforeCat(:,2), lambda);
r.catEffHC  = interp1(HCCatEff(:,1), HCCatEff(:,2), lambda);

r.emO2      = spline(O2TableBeforeCat(:,1), O2TableBeforeCat(:,2), lambda);
r.catEffO2  = 0;


