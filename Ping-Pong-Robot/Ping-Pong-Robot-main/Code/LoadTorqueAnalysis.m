clc 
clear all
close all
%%
%Caricamento workspace dal file "DeltaRobot.m"
load('tf005.mat');
A1 = zeros(3,istanti); 
O1 = zeros(3,istanti); 
O2 = zeros(3,istanti);
velO1 = zeros(3,istanti - 1); 
accO1 = zeros(3,istanti - 2); 
velO2 = zeros(3,istanti - 1); 
accO2 = zeros(3,istanti - 2); 
C1x = zeros(1,istanti - 2);
A2 = zeros(3,istanti); 
O21 = zeros(3,istanti); 
O22 = zeros(3,istanti);
velO21 = zeros(3,istanti - 1); 
accO21 = zeros(3,istanti - 2); 
velO22 = zeros(3,istanti - 1); 
accO22 = zeros(3,istanti - 2); 
C2x = zeros(1,istanti - 2);
A3 = zeros(3,istanti); 
O31 = zeros(3,istanti); 
O32 = zeros(3,istanti);
velO31 = zeros(3,istanti - 1); 
accO31 = zeros(3,istanti - 2); 
velO32 = zeros(3,istanti - 1); 
accO32 = zeros(3,istanti - 2); 
C3x = zeros(1,istanti - 2);


% Massa L [Kg]
mL = 1.305; 

% Massa l
ml = 0.433;

% Forza peso
forzaL = mL*[0; 0; -9.81];
forzal = ml*[0; 0; -9.81];


B1 = zeros(3,1);
B2 = zeros(3,1);
B3 = zeros(3,1);

Rz1 = [-1  0  0 ;
       0  -1  0 ;
       0   0  1];
% Rz1 = eye(3);
  
for j = 1:istanti
    A1(:,j) = Rz1*[0; -lSup*cos(angoli(1,j)); -lSup*sin(angoli(1,j))];
    O1(:,j) = Rz1*[0; -lSup*0.5*cos(angoli(1,j)); -lSup*0.5*sin(angoli(1,j))];
    P1sdrB1(:,j) = Rz1*( p1P + spostamento(:,j) - b1B );
    O2(:,j) = (A1(:,j) + P1sdrB1(:,j))/2;
end

for j = 1:3
    velO1(j,:) = diff(O1(j,:))./diff(tempo);
    accO1(j,:) = diff(velO1(j,:))./diff(tempo(1:istanti-1));
    velO2(j,:) = diff(O2(j,:))./diff(tempo);
    accO2(j,:) = diff(velO2(j,:))./diff(tempo(1:istanti-1));
end

accO1(:,tempoTc-2) = accO1(:,tempoTc-3);
accO1(:,tempoTc-1) = accO1(:,tempoTc);
accO1(:,2*tempoTc-2) = accO1(:,2*tempoTc-3);
accO1(:,2*tempoTc-1) = accO1(:,2*tempoTc);
accO2(:,tempoTc-2) = accO2(:,tempoTc-3);
accO2(:,tempoTc-1) = accO2(:,tempoTc);
accO2(:,2*tempoTc-2) = accO2(:,2*tempoTc-3);
accO2(:,2*tempoTc-1) = accO2(:,2*tempoTc);

Rz2 = [cos(-pi/3)   sin(-pi/3)    0;
       -sin(-pi/3)  cos(-pi/3)    0;
            0           0         1];
% Rz2 = eye(3);

for j = 1:istanti
    A2(:,j) = Rz2*[sqrt(3)*0.5*lSup*cos(angoli(2,j)); 0.5*lSup*cos(angoli(2,j)); -lSup*sin(angoli(2,j))];
    O21(:,j) = Rz2*[sqrt(3)*0.25*lSup*cos(angoli(2,j)); 0.25*lSup*cos(angoli(2,j)); -0.5*lSup*sin(angoli(2,j))];
    P2sdrB2(:,j) = Rz2*( p2P + spostamento(:,j) - b2B );
    O22(:,j) = (A2(:,j) + P2sdrB2(:,j))/2;
end

for j = 1:3
    velO21(j,:) = diff(O21(j,:))./diff(tempo);
    accO21(j,:) = round(diff(velO21(j,:))./diff(tempo(1:istanti-1)),5);
    velO22(j,:) = diff(O22(j,:))./diff(tempo);
    accO22(j,:) = round(diff(velO22(j,:))./diff(tempo(1:istanti-1)),5);
end
accO21(:,tempoTc-2) = accO21(:,tempoTc-3);
accO21(:,tempoTc-1) = accO21(:,tempoTc);
accO21(:,2*tempoTc-2) = accO21(:,2*tempoTc-3);
accO21(:,2*tempoTc-1) = accO21(:,2*tempoTc);
accO22(:,tempoTc-2) = accO22(:,tempoTc-3);
accO22(:,tempoTc-1) = accO22(:,tempoTc);
accO22(:,2*tempoTc-2) = accO22(:,2*tempoTc-3);
accO22(:,2*tempoTc-1) = accO22(:,2*tempoTc);

Rz3 = [cos(pi/3)   sin(pi/3)   0;
      -sin(pi/3)   cos(pi/3)   0;
            0           0      1];
% Rz3 = eye(3);  

for j = 1:istanti
    A3(:,j) = Rz3*[-sqrt(3)*0.5*lSup*cos(angoli(3,j)); 0.5*lSup*cos(angoli(3,j)); -lSup*sin(angoli(3,j))];
    O31(:,j) = Rz3*[-sqrt(3)*0.25*lSup*cos(angoli(3,j)); 0.25*lSup*cos(angoli(3,j)); -0.5*lSup*sin(angoli(3,j))];
    P3sdrB3(:,j) = Rz3*( p3P + spostamento(:,j) - b3B );
    O32(:,j) = (A3(:,j) + P3sdrB3(:,j))/2;
end

for j = 1:3
    velO31(j,:) = diff(O31(j,:))./diff(tempo);
    accO31(j,:) = round(diff(velO31(j,:))./diff(tempo(1:istanti-1)),5);
    velO32(j,:) = diff(O32(j,:))./diff(tempo);
    accO32(j,:) = round(diff(velO32(j,:))./diff(tempo(1:istanti-1)),5);
end

accO31(:,tempoTc-2) = accO31(:,tempoTc-3);
accO31(:,tempoTc-1) = accO31(:,tempoTc);
accO31(:,2*tempoTc-2) = accO31(:,2*tempoTc-3);
accO31(:,2*tempoTc-1) = accO31(:,2*tempoTc);
accO32(:,tempoTc-2) = accO32(:,tempoTc-3);
accO32(:,tempoTc-1) = accO32(:,tempoTc);
accO32(:,2*tempoTc-2) = accO32(:,2*tempoTc-3);
accO32(:,2*tempoTc-1) = accO32(:,2*tempoTc);

%% Inizializzazione variabili
eqCardinale = zeros(36,istanti-2);
sol1 = zeros(12,istanti-2);
sol2 = zeros(12,istanti-2);
sol3 = zeros(12,istanti-2);

%%
progressbar('Simulazione calcolo coppia per ogni istante di tempo');
for index = 1:istanti-2
    
    syms rB1x rB1y rB1z rB2x rB2y rB2z rB3x rB3y rB3z ... 
         ra1x ra1y ra1z ra2x ra2y ra2z ra3x ra3y ra3z ...
         tau1x tau1y tau1z tau2x tau2y tau2z tau3x tau3y tau3z ...
         fP1x fP1y fP1z fP2x fP2y fP2z fP3x fP3y fP3z real
    
    % Reazioni vincolari alla base
    rB1 = [rB1x; rB1y; rB1z];
    rB2 = [rB2x; rB2y; rB2z];
    rB3 = [rB3x; rB3y; rB3z];
    
    % Reazioni nei giunti universali
    ra1 = [ra1x; ra1y; ra1z];
    ra2 = [ra2x; ra2y; ra2z];
    ra3 = [ra3x; ra3y; ra3z];
    
    % Coppia degli attuatori
    tau1 = [tau1x; tau1y; tau1z];
    tau2 = [tau2x; tau2y; tau2z];
    tau3 = [tau3x; tau3y; tau3z];
    
    % Reazioni sulla platform
    fP1 = [fP1x; fP1y; fP1z];
    fP2 = [fP2x; fP2y; fP2z];
    fP3 = [fP3x; fP3y; fP3z];
    
    % Eq. Cardinali per il primo link, gamba 1 (_1)
    eqCardinale1Link1_1 = forzaL + ra1 + rB1 - mL*accO1(:,index) == 0;
    eqCardinale2Link1_1 = tau1 + cross((A1(:,index) - O1(:,index)),ra1) + cross((B1 - O1(:,index)),rB1) == 0;

    % Eq. Cardinali per il secondo link, gamba 1 (_1)
    eqCardinale1Link2_1 = forzal - ra1 + fP1 - ml*accO2(:,index) == 0;
    eqCardinale2Link2_1 = cross((A1(:,index) - O2(:,index)),-ra1) + cross((P1sdrB1(:,index) - O2(:,index)),fP1) == 0;
    
    % Eq. Cardinali per il primo link, gamba 2 (_2)
    eqCardinale1Link1_2 = forzaL + ra2 + rB2 - mL*accO21(:,index) == 0;
    eqCardinale2Link1_2 = tau2 + cross((A2(:,index) - O21(:,index)),ra2) + cross((B2 - O21(:,index)),rB2) == 0;

    % Eq. Cardinali per il secondo link, gamba 2 (_2)
    eqCardinale1Link2_2 = forzal - ra2 + fP2 - ml*accO22(:,index) == 0;
    eqCardinale2Link2_2 = cross((A2(:,index) - O22(:,index)),-ra2) + cross((P2sdrB2(:,index) - O22(:,index)),fP2) == 0;
    
    % Eq. Cardinali per il primo link, gamba 3 (_3)
    eqCardinale1Link1_3 = forzaL + ra3 + rB3 - mL*accO31(:,index) == 0;
    eqCardinale2Link1_3 = tau3 + cross((A3(:,index) - O31(:,index)),ra3) + cross((B3 - O31(:,index)),rB3) == 0;

    % Eq. Cardinali per il secondo link, gamba 3 (_3)
    eqCardinale1Link2_3 = forzal - ra3 + fP3 - ml*accO32(:,index) == 0;
    eqCardinale2Link2_3 = cross((A3(:,index) - O32(:,index)),-ra3) + cross((P3sdrB3(:,index) - O32(:,index)),fP3) == 0;

    % Risoluzione Sistema
    [A,B] = equationsToMatrix(eqCardinale1Link1_1(1),eqCardinale1Link1_1(2),eqCardinale1Link1_1(3), ...
                              eqCardinale2Link1_1(1),eqCardinale2Link1_1(2),eqCardinale2Link1_1(3), ...
                              eqCardinale1Link2_1(1),eqCardinale1Link2_1(2),eqCardinale1Link2_1(3), ...
                              eqCardinale2Link2_1(1),eqCardinale2Link2_1(2),eqCardinale2Link2_1(3), ...
                              eqCardinale1Link1_2(1),eqCardinale1Link1_2(2),eqCardinale1Link1_2(3), ...
                              eqCardinale2Link1_2(1),eqCardinale2Link1_2(2),eqCardinale2Link1_2(3), ...
                              eqCardinale1Link2_2(1),eqCardinale1Link2_2(2),eqCardinale1Link2_2(3), ...
                              eqCardinale2Link2_2(1),eqCardinale2Link2_2(2),eqCardinale2Link2_2(3), ...
                              eqCardinale1Link1_3(1),eqCardinale1Link1_3(2),eqCardinale1Link1_3(3), ...
                              eqCardinale2Link1_3(1),eqCardinale2Link1_3(2),eqCardinale2Link1_3(3), ...
                              eqCardinale1Link2_3(1),eqCardinale1Link2_3(2),eqCardinale1Link2_3(3), ...
                              eqCardinale2Link2_3(1),eqCardinale2Link2_3(2),eqCardinale2Link2_3(3), ...
                             [rB1x, rB1y, rB1z, ra1x, ra1y, ra1z, tau1x, tau1y, tau1z, fP1x, fP1y, fP1z, ...
                              rB2x, rB2y, rB2z, ra2x, ra2y, ra2z, tau2x, tau2y, tau2z, fP2x, fP2y, fP2z, ...
                              rB3x, rB3y, rB3z, ra3x, ra3y, ra3z, tau3x, tau3y, tau3z, fP3x, fP3y, fP3z]);
                          
   A = double(A);
   B = double(B);
   gamba1 = A(1:12,1:12);
   gamba2 = A(13:24,13:24);
   gamba3 = A(25:36,25:36);
   
   [acol1,icol1]=licols(gamba1,0.1);
   [acol2,icol2]=licols(gamba2,0.1);
   [acol3,icol3]=licols(gamba3,0.1);

   
   TyNull = [0 0 0 0 0 0 0 1 0 0 0 0];
   acolnew1 = [acol1;TyNull];
   acolnew2 = [acol2;TyNull];
   acolnew3 = [acol3;TyNull];
   
   sol1(:,index) = linsolve(acolnew1,B(1:12));
   sol2(:,index) = linsolve(acolnew2,B(13:24));
   sol3(:,index) = linsolve(acolnew3,B(25:36));
   
   % Ricavo la coppia dal vettore soluzione
   C1(:,index) = sol1(7:9,index);
   C2(:,index) = sol2(7:9,index);
   C3(:,index) = sol3(7:9,index);
   
   % Verifica che siano rispettate tutte le equazioni cardinali
   % per ogni gamba
   eqCardinale(1:12,index) = gamba1*sol1(:,index) - B(1:12);
   eqCardinale(13:24,index) = gamba2*sol2(:,index) - B(13:24);
   eqCardinale(25:36,index) = gamba3*sol3(:,index) - B(25:36);
   
   progressbar(index/(istanti-2))
end
%% Plot coppia
figure(1)
plot(tempo(1:istanti-2),C1(1,:),'b-');
hold on
plot(tempo(1:istanti-2),C2(1,:),'g-');
hold on
plot(tempo(1:istanti-2),C3(1,:),'r-');
hold on
title('Trend of torque for each leg over the time');
grid on
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('Leg 1','Leg 2','Leg 3');
%%
% Calcolo potenza media assorbita
P1m = sum(C1(1,:).*velocitaAngolari(1,3:istanti))/length(C1)
P2m = sum(C2(1,:).*velocitaAngolari(2,3:istanti))/length(C2)
P3m = sum(C3(1,:).*velocitaAngolari(3,3:istanti))/length(C3)
%% Verifica che tutte le equazioni cardinali impostate siano rispettate
% figure
% for i = 1 : 36
%     plot(tempo(1:istanti-2),eqCardinale(i,:),'k-');
%     hold on
% end
% title('equ. cardinali in funzione del tempo');
% grid on
% xlabel('tempo [s]');
% ylabel('val. equ cardinali');