%Risoluzione problema della pallina da ping pong

clc
clear all
close all

%% Definizione parametri di input
%massa della pallina
massaPallina = 0.0027;      

%accelerazione gravitazionale
g = 9.81;                   

%dimensioni del campo
fcx = 2.74;
fcy = 1.52;
fcz = 0;
f = [fcx;fcy;fcz];

%Punto di target
tx = 2.64;         
ty = 0.45;
tz = 0;         
target = [tx;ty;tz];

%% Risoluzione

%Definizione del numero di punti di impatto sul campo
campioniPerRiga = 3;
campioniPerColonna = 5;
numeroCampioni = campioniPerRiga*campioniPerColonna;

% Inizializzazione strutture dati che conterranno la soluzione del problema
vRacchetta = zeros(numeroCampioni,3);
azimutRacchetta = zeros(numeroCampioni,1);
inclinationRacchetta = zeros(numeroCampioni,1);
vPostPallina = zeros(3,numeroCampioni);

% Definizione parametri urto
e = 0.81;
kp = 0.0019;
a = kp/massaPallina;

%velocità della pallina in ingresso
vPrePallina = [-8.5;-2;2.2];
% eta = atan2(ty - posizioney(7),tx-posizionex(7));
% vPrePallina = [-8.5*cos(eta);-8.5*sin(eta);2.22];

posizionez = ones(numeroCampioni,1)*0.28;
posizionex = zeros(numeroCampioni,1);
posizioney = zeros(numeroCampioni,1);

% Definizione dei punti d'impatto
xSpace = linspace(0,0.4,campioniPerRiga);
ySpace = linspace(0,-0.76,campioniPerColonna);
k = 1;

for i = 1 : campioniPerRiga
    for j = 1 : campioniPerColonna
         posizionex(k) = xSpace(i);
         posizioney(k) = ySpace(j);
         k = k + 1;
    end
end



for i = 1:numeroCampioni
    px = posizionex(i);
    py = posizioney(i);
    pz = posizionez(i);
    p = [px;py;pz];
        
    rx = 1.37;                      %Posizione della rete
    ry = py;
    rz = 0.1725;
    r = [rx;ry;rz];
    
    % Componente su z della pallina
    vPostPallina(3,i) = vPrePallina(3)*( 1 - a ); 
    hMax = (vPostPallina(3,i)^2)/(2*g) + pz;
    
    % Angolo di azimut della pallina prima dell'urto
    azimutPreP = atan2(-vPrePallina(2),-vPrePallina(1));
    
    % Risoluzione traiettoria
    syms t
    equ = (0.02-pz)+0.5*g*t^2-vPostPallina(3,i)*t;
    sol = solve(equ==0,t);
    t = double(sol);
    indice = find(t > 0);
    time = t(indice);
    vettoreModulo = [tx-px;ty-py];
    vPostPallin = norm(vettoreModulo)/time;
    
    % Angolo di azimut della pallina dopo l'urto
    azimutPostP = atan2(vettoreModulo(2),vettoreModulo(1));
    
    % Angolo di azimut rispetto all'asse ortogonale al piano racchetta, 
    % dei 2 precedentemente calcolati
    alfa = ( azimutPostP  + azimutPreP )/2;
    vPostPallina(1:2,i) = [vPostPallin*cos(azimutPostP);vPostPallin*sin(azimutPostP)];
    
    %Analisi urto
    syms vRacchettax vRacchettay
    
    % Matrice di rotazione che ci permette di passare dal sdr fisso
    % a quello solidale alla racchetta
    R = [cos(alfa)  sin(alfa)  0;
         -sin(alfa) cos(alfa)  0;
             0           0     1];
   
    Avv = [-e 0 0;0 1-a 0;0 0 1-a];
    
    Vrac = [vRacchettax; vRacchettay; 0];
    
    % Risoluzione Urto
    equ = R*(vPostPallina(:,i) - Vrac) == Avv*R*(vPrePallina - Vrac);     
    vRacc = solve(equ, vRacchettax, vRacchettay);
    soluzione = double(struct2array(vRacc));
    
    azimutRacchetta(i) = rad2deg(alfa) - 90;
    vRacchetta(i,:) = [soluzione(1:2), 0];
end
%% Salvataggio dei dati
A = [posizionex, posizioney, posizionez, vRacchetta, inclinationRacchetta, azimutRacchetta, target'.*ones(numeroCampioni,1), vPostPallina'];

filename = 'casiSimulati.xlsx';
writematrix(A,filename,'Sheet',1);

%% Plot Traiettoria

p = [posizionex(8); posizioney(8); posizionez(8)];

initial = [p;vPostPallina(:,8)];
step = 0.0001;
movimentoPredetto2 = risoluzioneMovimento(massaPallina,g,step,time,initial);
figure(1)
plot3(p(1), p(2), p(3),'bo','MarkerSize',11);
hold on
plot3(tx,ty,tz,'r*');
% legend('Impact zone','Target point');

for i = 30 : round(length(movimentoPredetto2)/90) : length(movimentoPredetto2)
    hold on
    plot3(movimentoPredetto2(i,1),movimentoPredetto2(i,2),movimentoPredetto2(i,3),'o','Color','#77AC30','MarkerSize',11);
end
plot3(movimentoPredetto2(length(movimentoPredetto2),1),movimentoPredetto2(length(movimentoPredetto2),2),movimentoPredetto2(length(movimentoPredetto2),3),'o','Color','#77AC30','MarkerSize',11);
hold on;
plotCampo();
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
title('Trajectory');
axis([-0.20 3 -0.8 0.8 -0.1 1]);
legend('Impact zone','Target point','Trajectory'); 
%% Plot
% k è un vettore che identifica i punti da analizzare sul plot del campo
k = 8;
for i = 1 : length(k)
    figure(1+i);
    v = pnc(filename,vPrePallina,k(i));
end
