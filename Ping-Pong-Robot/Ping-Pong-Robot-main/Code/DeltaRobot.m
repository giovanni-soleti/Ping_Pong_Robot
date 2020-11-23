clc
clear all
close all

%% Parametri geometrici
% NOTAZIONE: prima parola definisce il vertice, lettera maiuscola il sdr
% in cui è definito.

% b --> riferito alla base
% p --> riferito alla piattaforma mobile del delta robot

% distanza dalla mediana al centro del lato del tr. equilatero (base)
wb = 0.164; 
% lunghezza lato tr. equ
sb = 0.567;
% distanza dal centro dal vertice del tr. equilatero (base)
ub = 0.327; 
% distanza dalla mediana al centro del lato del tr. equilatero (platform)
wp = 0.022; 
% lunghezza lato tr. equ
sp = 0.076; 
% distanza dal centro del vertice del tr. equilatero (platform)
up = 0.044; 

%altezza dal tavolo della base
altezzaDalTavolo = 1.3;
%posizione del robot rispetto al tavolo
distanzaDalFondo = 0.32;
posizioneBase = [distanzaDalFondo;0;altezzaDalTavolo];

%lunghezza del manico
dx = 1; %1 o -1 a seconda che l'impatto sia a destra o sinistra
impattoRacchetta = dx*(0.1 + 0.075);

% posizione degli attuatori nel sdr base
b1B = [0; -wb; 0];                             
b2B = [sqrt(3)*0.5*wb; 0.5*wb; 0];  
b3B = [-sqrt(3)*0.5*wb; 0.5*wb; 0]; 
jointBaseB = [b1B,b2B,b3B];

% posizione dei vertici della base
vb1B = [sb*0.5; -wb; 0];            
vb2B = [0; ub; 0];                  
vb3B = [-sb*0.5; -wb; 0];
baseB = [vb1B, vb2B, vb3B];

% posizione dei vertici della piattaforma nel sdr piattaforma
p1P = [0; -up; 0];                 
p2P = [sp*0.5; wp; 0];              
p3P = [-sp*0.5; wp; 0];

% lunghezza dell' arto inferiore
lInf = 1.244;

% lunghezza arto superiore
lSup = 0.524;

% plot del campo
xInizio = 0;
xFine = 2.74;
xRete = 1.37;
yInizio = -0.76;
yFine = 0.76;
zRete = 0.1725;

%load simulazione from the script "OutgoingTrajectory_ReboundModel.m"
sim = xlsread('casiSimulati.xlsx');

%% Risoluzione cinematica inversa

numeroCampioni = length(sim(:,1));

% Caso considerato
i = 8;

% sdr Tavolo
xsdrTavolo = sim(i,1);
ysdrTavolo = sim(i,2);
zsdrTavolo = sim(i,3);

% sdr Robot
xsdrRobot = xsdrTavolo - posizioneBase(1);
ysdrRobot = ysdrTavolo - posizioneBase(2);
zsdrRobot = zsdrTavolo - posizioneBase(3);
inclinationP = deg2rad(sim(i,7));
azimutP = deg2rad(sim(i,8));

velocitaAngolari=zeros(3,numeroCampioni);

% Parametri Triettoria
tf = 0.02;
istanti = 200;

% angoli attuati
angoli = zeros(3,istanti);

%figure(1)
%figure('units','normalized','outerposition',[0 0 1 1])

%Matrice di rotazione tra sdr BASE e sdr PLATFORM
R_Y = [cos(inclinationP) 0 -sin(inclinationP) ;
            0            1           0        ;
       sin(inclinationP) 0 cos(inclinationP)]';

%Matrice di rotazione Z 
R_Z = [cos(azimutP) -sin(azimutP)     0;
       sin(azimutP)  cos(azimutP)     0;
            0               0         1];

% Punto di impatto con la racchetta (manico + raggio circonferenza)
puntoDiImpatto = R_Z * [0.175; 0; 0];

% Coordinate Platform per impattare la pallina
xPlat = xsdrRobot - puntoDiImpatto(1);
yPlat = ysdrRobot - puntoDiImpatto(2);
zPlat = zsdrRobot - puntoDiImpatto(3);
platformB = [xPlat; yPlat; zPlat];

% Velocità lineari dell'EE all'impatto
xPunto = sim(i,4);
yPunto = sim(i,5);
zPunto = sim(i,6);
v = [xPunto; yPunto; zPunto];

% Pianificazione della traiettoria
[posInit, posFin, tc, acc, soglia] = traiettoriaEE_td(platformB,tf, v);
tempo = linspace(0,tf,istanti);
tempoTc = round((tc/tf)*istanti);

% accelerazioni dell'EE in ogni istante
accelerazione(1,:) = acc(1)*[ones(1,tempoTc), zeros(1,istanti-2*tempoTc), -1*ones(1,tempoTc)];
accelerazione(2,:) = acc(2)*[ones(1,tempoTc), zeros(1,istanti-2*tempoTc), -1*ones(1,tempoTc)];
accelerazione(3,:) = acc(3)*[ones(1,tempoTc), zeros(1,istanti-2*tempoTc), -1*ones(1,tempoTc)];

spostamento = zeros(3,istanti);
velocita = zeros(3,istanti);

%definizione nel tempo delle caratteristiche precedentemente determinate
for j = 1 : 3
    for k = 1 : istanti
        if ( k < tempoTc )
            velocita(j,k) = accelerazione(j,k)*tempo(k);
            spostamento(j,k) = 0.5*accelerazione(j,k)*tempo(k)^2 + posInit(j);
        elseif( k >= tempoTc && k <= (istanti - tempoTc) )
            velocita(j,k) = v(j);
            spostamento(j,k) = spostamento(j,tempoTc-1) + v(j)*tempo(k - tempoTc + 1);
        else 
            velocita(j,k) = v(j) + accelerazione(j,k)*tempo( k - ( istanti - tempoTc) );
            spostamento(j,k) = spostamento(j,(istanti - tempoTc)) + v(j)*tempo( k - ( istanti - tempoTc)) + 0.5*accelerazione(j,k)*tempo( k - ( istanti - tempoTc))^2;
        end
    end
end

% Posizione dei vertici della piattaforma nel sdr Piattaforma
verticiP = R_Y * [p1P,p2P,p3P];

for j = 1 : istanti
    x = spostamento(1,j);
    y = spostamento(2,j);
    z = spostamento(3,j);
    vx = velocita(1,j);
    vy = velocita(2,j);
    vz = velocita(3,j);
    
    % Per ogni gamba l'equ impostata è: 
    % E(i)*cos(theta_i) + F(i)*sin(theta_i) + G(i) = 0
    E = zeros(1,3);
    E(1) = 2*lSup*(y + verticiP(2,1) + wb);
    E(2) = -sqrt(3)*lSup*( x + verticiP(1,2) - 0.5*sqrt(3)*wb ) ...
            - lSup*( y + verticiP(2,2) - 0.5*wb);
    E(3) = sqrt(3)*lSup*( x + verticiP(1,3) + 0.5*sqrt(3)*wb ) ...
            - lSup*( y + verticiP(2,3) - 0.5*wb );

    F = zeros(1,3);
    F(1) = 2*lSup*( z + verticiP(3,1));
    F(2) = 2*lSup*( z + verticiP(3,2));
    F(3) = 2*lSup*( z + verticiP(3,3));

    G = zeros(1,3);
    G(1) = (x + verticiP(1,1))^2 + ( x + verticiP(2,1) + wb)^2 ...
            + ( z + verticiP(3,1) )^2 + lSup^2 - lInf^2;
    G(2) = ( x + verticiP(1,2) - sqrt(3)*0.5*wb )^2 + ( y + verticiP(2,2)...
            - 0.5*wb)^2 + ( z + verticiP(3,2) )^2 + lSup^2 - lInf^2;
    G(3) = ( x + verticiP(1,3) + sqrt(3)*0.5*wb )^2 + ( y + verticiP(2,3)...
            - 0.5*wb)^2 + ( z + verticiP(3,3) )^2 + lSup^2 - lInf^2;

    t = zeros(3,1);

    for k = 1 : 3
        t(k) = (-F(k) - sqrt(E(k)^2 + F(k)^2 - G(k)^2))/(G(k) - E(k));
        angoli(k,j) = 2*atan(t(k));      
    end
    
    if (imag(angoli(:,j)))
        error(sprintf('Error: Cannot reach this point: [%d,%d,%d]',x,y,z));
    end

    % Calcolo Jacobiano vLin*xdot = vAng*tethadot
    vLin = [(x + verticiP(1,1)), ( lSup*cos(angoli(1)) + y + verticiP(2,1) + wb), ( lSup*sin(angoli(1)) + verticiP(3,1) + z );
        ( 2* ( x + verticiP(1,2) - sqrt(3)*0.5*wb ) - sqrt(3)*lSup*cos(angoli(2))), ( 2*( y + verticiP(2,2) - 0.5*wb) - lSup*cos(angoli(2))), ( 2*lSup*sin(angoli(2)) + 2*( z + verticiP(3,2)));
        ( 2* ( x + verticiP(1,3) + sqrt(3)*0.5*wb ) + sqrt(3)*lSup*cos(angoli(3))), ( 2*( y + verticiP(2,3) - 0.5*wb) - lSup*cos(angoli(3))), ( 2*lSup*sin(angoli(3)) + 2*( z + verticiP(3,3)))];

    vAng = [lSup*(( y + verticiP(2,1) + wb )*sin(angoli(1)) - ( verticiP(3,1) + z )*cos(angoli(1))) 0 0;
        0 ((- sqrt(3)*lSup*( x + verticiP(1,2) - sqrt(3)*0.5*wb ) - lSup*( y + verticiP(2,2) - 0.5*wb )*sin(angoli(2)) - 2*lSup*( verticiP(3,2) + z )*cos(angoli(2)))) 0;
        0 0 ((+ sqrt(3)*lSup*( x + verticiP(1,3) + sqrt(3)*0.5*wb ) - lSup*( y + verticiP(2,3) - 0.5*wb )*sin(angoli(3)) - 2*lSup*( verticiP(3,3) + z )*cos(angoli(3))))];

    velocitaAngolari(:,j) = vAng\vLin*[vx;vy;vz];
     
%      clf
%      result = plotDelta(lSup,lInf,angoli(:,j),[x;y;z],jointBaseB,verticiP,baseB,R_Y,posizioneBase,R_Z);
%      hold on
%      plotCampo(sim);
%      pause(0.01)
%      hold on
end

file = sprintf('Simulazione_numero_%d.mat',i);
save(file);

spostamento(1,:) = spostamento(1,:) + distanzaDalFondo;
spostamento(3,:) = spostamento(3,:) + altezzaDalTavolo;
%% Analisi End-Effector + Giunti attuati
fig2 = figure(2);
campioni = 1:istanti;
plot(tempo,rad2deg(angoli(1,1:istanti)),'b-',tempo,rad2deg(angoli(2,1:(istanti))),'g-',tempo,rad2deg(angoli(3,1:(istanti))),'r-');
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Trend of angle over the time')
axis([0 tf min(min(rad2deg(angoli)))-5 max(max(rad2deg(angoli)))+5]);
grid on
legend('Actuated joint 1','Actuated joint 2','Actuated joint 3');
saveas(fig2,sprintf('AngAtT_%d.png',i));
%%
fig3 = figure(3);
plot(tempo,velocitaAngolari(1,1:istanti),'b-',tempo,velocitaAngolari(2,1:istanti),'g-',tempo,velocitaAngolari(3,1:istanti),'r-');
xlabel('Time [s]');
ylabel('Angular speed [rad/s]');
title('Trend of angular speed over the time')
axis([0 tf min(min(velocitaAngolari))-1 max(max(velocitaAngolari))+1]);
grid on
legend('Angular speed joint 1','Angular speed joint 2','Angular speed joint 3');
saveas(fig3,sprintf('velAngT_%d.png',i));
%%
fig5 = figure(5);
plot(tempo,spostamento(1,:),'r-',tempo,spostamento(2,:),'b-',tempo,spostamento(3,:),'g-');
grid on
xlabel('Time [s]');
ylabel('Movement [m]');
axis([0 tf min(min(spostamento))-0.2 max(max(spostamento))+0.2]);
title('Movement of End Effector over the time');
legend('x','y','z');
saveas(fig5,sprintf('sposT_%d.png',i));
%% 
fig6 = figure(6);
plot(tempo,velocita(1,:),'r-',tempo,velocita(2,:),'b-',tempo,velocita(3,:),'g-');
grid on
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([0 tf min(min(velocita))-0.2 max(max(velocita))+0.2]);
title('Speed diagram of End Effector over the time');
legend('x','y','z');
saveas(fig6,sprintf('velT_%d.png',i));
%%
fig7 = figure(7);
plot(tempo, accelerazione(1,:) ,'r-',tempo,accelerazione(2,:),'b-',tempo,accelerazione(3,:),'g-');
grid on
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
axis([0 tf min(min(accelerazione))-5 max(max(accelerazione))+5]);
title('Acceleration diagram of End Effector over the time');
legend('x','y','z');
saveas(fig7,sprintf('AccT_%d.png',i));
%%
fig8 = figure(8);
plot3(spostamento(1,:), spostamento(2,:), spostamento(3,:),'r-');
grid on
xlabel('x[m]');
ylabel('y');
zlabel('z');
title('Spostamento dell End Effector');
minPar = min(soglia);
maxPar = max(soglia);
axis([spostamento(1,round(istanti*0.5))-maxPar spostamento(1,round(istanti*0.5))+maxPar spostamento(2,round(istanti*0.5))-maxPar spostamento(2,round(istanti*0.5))+maxPar spostamento(3,round(istanti*0.5))-maxPar spostamento(3,round(istanti*0.5))+maxPar]);
saveas(fig8,sprintf('SpostamentoEEspace_%d.png',i));
%%
fig9 = figure(9);
velocitaAngolariRPM = velocitaAngolari*(60/2*pi);
plot(tempo,velocitaAngolariRPM(1,1:istanti),'b-',tempo,velocitaAngolariRPM(2,1:istanti),'g-',tempo,velocitaAngolariRPM(3,1:istanti),'r-');
xlabel('Time [s]');
ylabel('Angular speed [RPM]');
title('Trend of angular speed in RPM of actuated joint')
axis([0 tf min(min(velocitaAngolariRPM))-1 max(max(velocitaAngolariRPM))+1]);
grid on
legend('Angular speed joint 1','Angular speed joint 2','Angular speed joint 3');
saveas(fig9,sprintf('velAngTRPM_%d.png',i));
%%
fig10 = figure(10);
plot(spostamento(1,:),spostamento(2,:),'r-');
grid on
xlabel('x[m]');
ylabel('y[m]');
title('Movement of End Effector');
minPar = min(soglia);
maxPar = max(soglia);
axis([spostamento(1,round(istanti*0.5))-maxPar spostamento(1,round(istanti*0.5))+maxPar spostamento(2,round(istanti*0.5))-maxPar spostamento(2,round(istanti*0.5))+maxPar]);
saveas(fig10,sprintf('EE_PianoXY_%d.png',i));
set(gca,'xdir','reverse','ydir','reverse')


