function [result] = plotDelta(lSup,lInf,teta,platformB,jointBaseB,verticiP,baseB,R_Y,posizioneBase,R_Z)


artoSuperioreX = zeros(3,2);
artoSuperioreY = zeros(3,2);
artoSuperioreZ = zeros(3,2);
artoInferioreX = zeros(3,2);
artoInferioreY = zeros(3,2);
artoInferioreZ = zeros(3,2);

jointBaseB = jointBaseB + posizioneBase;
baseB = baseB + posizioneBase;
platformB = platformB + posizioneBase;

% Costruzione della racchetta
% Definizione geometria della racchetta
lunghezzaManico = 0.1;
raggio = 0.075;
% Discretizzazione della circonferenza
nc = 50;

manico = R_Z*[lunghezzaManico;0;0];
centroCirconferenza = R_Z * [(lunghezzaManico + raggio);0;0];
y = zeros(nc,1);
t = 0:(2*pi)/(nc-1):2*pi;
x = raggio*sin(t);
z = raggio*cos(t);

lSup1B = [0;-lSup*cos(teta(1));-lSup*sin(teta(1))];
lSup2B = [sqrt(3)*0.5*lSup*cos(teta(2));0.5*lSup*cos(teta(2));-lSup*sin(teta(2))];
lSup3B = [-sqrt(3)*0.5*lSup*cos(teta(3));0.5*lSup*cos(teta(3));-lSup*sin(teta(3))];
lSupB  = [lSup1B,lSup2B,lSup3B];

lInf1B = platformB + verticiP(:,1) - jointBaseB(:,1) - lSup1B;
lInf2B = platformB + verticiP(:,2) - jointBaseB(:,2) - lSup2B;
lInf3B = platformB + verticiP(:,3) - jointBaseB(:,3) - lSup3B;


jointLegsB = jointBaseB + lSupB;

% vertici della piattaforma nel sdr piattaforma
p1B = platformB + verticiP(:,1);
p2B = platformB + verticiP(:,2);
p3B = platformB + verticiP(:,3);
jointPiattaformaB = [p1B,p2B,p3B];

triangoloSupX = [baseB(1,1) baseB(1,2); baseB(1,2) baseB(1,3); baseB(1,3) baseB(1,1)];
triangoloSupY = [baseB(2,1) baseB(2,2); baseB(2,2) baseB(2,3); baseB(2,3) baseB(2,1)];
triangoloSupZ = [baseB(3,1) baseB(3,2); baseB(3,2) baseB(3,3); baseB(3,3) baseB(3,1)];

triangoloInfX = [p1B(1) p2B(1); p2B(1) p3B(1); p3B(1) p1B(1)];
triangoloInfY = [p1B(2) p2B(2); p2B(2) p3B(2); p3B(2) p1B(2)];
triangoloInfZ = [p1B(3) p2B(3); p2B(3) p3B(3); p3B(3) p1B(3)];


manicoRacchettaX = [platformB(1); platformB(1) + manico(1)]; % 10 cm
manicoRacchettaY = [platformB(2); platformB(2) + manico(2)];
manicoRacchettaZ = [platformB(3); platformB(3) + manico(3)];
platformB(3)

% 15 cm diametro, r = 0.075


view(3);

for i = 1 : 3
    artoSuperioreX = [jointBaseB(1,i) jointLegsB(1,i)];
    artoInferioreX = [jointLegsB(1,i) jointPiattaformaB(1,i)];
    artoSuperioreY = [jointBaseB(2,i) jointLegsB(2,i)];
    artoInferioreY = [jointLegsB(2,i) jointPiattaformaB(2,i)];
    artoSuperioreZ = [jointBaseB(3,i) jointLegsB(3,i)];
    artoInferioreZ = [jointLegsB(3,i) jointPiattaformaB(3,i)];
    line(artoSuperioreX,artoSuperioreY,artoSuperioreZ,'Color','green','LineWidth',2);
    hold on
    line(artoInferioreX,artoInferioreY,artoInferioreZ,'Color','red','LineWidth',2);
    hold on
end

line(triangoloSupX,triangoloSupY,triangoloSupZ,'Color','blue','LineWidth',2);
fill3(triangoloSupX,triangoloSupY,triangoloSupZ,'b');
hold on
line(triangoloInfX,triangoloInfY,triangoloInfZ,'Color','blue','LineWidth',2);
fill3(triangoloInfX,triangoloInfY,triangoloInfZ, 'b')
hold on
line(manicoRacchettaX,manicoRacchettaY,manicoRacchettaZ,'Color','black','LineWidth',2);
hold on

for i = 1 : 3
    plot3(jointBaseB(1,i),jointBaseB(2,i),jointBaseB(3,i),'ro');
    hold on
    plot3(jointLegsB(1,i),jointLegsB(2,i),jointLegsB(3,i),'ro');
    hold on
    plot3(jointPiattaformaB(1,i),jointPiattaformaB(2,i),jointPiattaformaB(3,i),'ro');
    hold on
end

plot3(posizioneBase(1),posizioneBase(2),posizioneBase(3),'b*',platformB(1),platformB(2),platformB(3),'b*');
hold on
for i = 1 : nc
    racchettaRuotata = R_Z * R_Y' * [x(i); y(i); z(i)];
    x(i) = racchettaRuotata(1);
    y(i) = racchettaRuotata(2);
    z(i) = racchettaRuotata(3);
end
plot3(x + centroCirconferenza(1) + platformB(1) , y + centroCirconferenza(2) + platformB(2), z + platformB(3) + centroCirconferenza(3), 'k');
fill3(x + centroCirconferenza(1) + platformB(1) , y + centroCirconferenza(2) + platformB(2), z + platformB(3) + centroCirconferenza(3), 'r');
hold on
line([posizioneBase(1) platformB(1)],[posizioneBase(2) platformB(2)],[posizioneBase(3) platformB(3)],'Color','Red','LineStyle','--');

title('Delta Robot');
axis([-0.4 1.4 -1.2 1.2 0 1.9]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
% view(90,10)  % XY
view(270,10)  % XY
grid on;

result = 1;

end