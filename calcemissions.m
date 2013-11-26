function [EmissionmassBeforeCat,EmissionmassAfterCat] = ...
    calcemissions(tp, lambda3, dist, mairc,mfc, lightOffTime)
% [EmissionmassBeforeCat,EmissionmassAfterCat] = 
% calcemissions(t, lambda, dist, mairc, mfc, lightOffTime)
%
% Funktionen calcEmissions använder följande  indata för att
% bestämma emissionsnivån före och efter cat.
% Indata:
%	t      tillhörande tidsvektor med tidpunkter för varje sampel [s]
%       lambda  en vektor med labmdavärden för varje tidpunkt i en körcykeln [s]
%	dist	den körda distansen i m vid varje sampel.
%	mairc	vektor med luftflöde in i cylinder för varje tidpunkt [kg/s]
%	mfc	-"-        bränsleföde   -"- [kg/s]
%       lightOffTime Tiden i sekunder tills katalysatorn startar [s]
% Utdata:
%	emissionmassBC	emissionsnivå för [CO NOx HC] i [g/km] före cat
%       emissionmassAC	emissionsnivå för [CO NOx HC] i [g/km] efter cat
%
%  Av : Ingemar Andersson
%       Per Andersson
%
%  $Date: 2004/07/06 09:37:50 $
%  $Revision: 1.1.1.1 $

% Konvertera från SI-enheter
dist  = dist/1e3;
mairc = mairc * 1e3;
mfc   = mfc * 1e3;

tic;

%disp('Beräknar emissioner')
result = emissions(lambda3);
%disp('Utvärderar')

% Utvärderar data

loff = tp > lightOffTime; % Ger alla tidpunkter större än light off

MM = [28 14+16 16]; % Molmassor. Kompenserar för att emissions
% returnerar tal i procent för CO samt promille för NOx och HC.
% Observera att O2 och H2 inte används

MCO  = 12+16;
MNOx = 14+16;
MHC  = 12+4;
MO2  = 32;

% Total massa emissioner
EMF =  mairc + mfc;

Mtot = 28.5; % Ur Heywood.

len = length(result.emCO);

resultCO  = EMF .* (result.emCO*MCO)/(Mtot)/100;
resultNOx = EMF .* (result.emNOx*MNOx)/(Mtot)/1000;
resultHC  = EMF .* (result.emHC*MHC)/(Mtot)/1000;
resultO2  = EMF .* (result.emO2*MO2/Mtot);

EmissionmassBeforeCat = [resultCO resultNOx resultHC]'*[diff(tp); 0]/dist(length(dist));

disp('Emissioner före katalysator:')
disp(sprintf('\t CO  : %1.2f [g/km]', EmissionmassBeforeCat(1)))
disp(sprintf('\t HC  : %1.2f [g/km]', EmissionmassBeforeCat(3)))
disp(sprintf('\t NOx : %1.2f [g/km]', EmissionmassBeforeCat(2)))


%result2(i,:) = EMF(i) * (XvolCat(i,:).*MM)/(Mtot);
resultCO  = EMF .* (result.emCO*MCO)/(Mtot)/100.*(1-result.catEffCO.*loff);
resultNOx = EMF .* (result.emNOx*MNOx)/(Mtot).*(1-result.catEffNOx.*loff)/1000;
resultHC  = EMF .* (result.emHC*MHC)/(Mtot).*(1-result.catEffHC.*loff)/1000;
resultO2  = EMF .* (result.emO2*MO2/Mtot);
 
EmissionmassAfterCat = [resultCO resultNOx resultHC]'*[diff(tp); ...
0]/dist(length(dist));
disp(' ')

disp('Emissioner efter katalysator:')
disp(sprintf('\t CO  : %1.2e [g/km]\tGränsvärde EURO 3 : 2.3  [g/km]   EURO 4 : 1.0  [g/km]', EmissionmassAfterCat(1)))
disp(sprintf('\t HC  : %1.2e [g/km]\tGränsvärde EURO 3 : 0.20 [g/km]   EURO 4 : 0.10 [g/km]', EmissionmassAfterCat(3)))
disp(sprintf('\t NOx : %1.2e [g/km]\tGränsvärde EURO 3 : 0.15 [g/km]   EURO 4 : 0.08 [g/km]', EmissionmassAfterCat(2)))
%toc

