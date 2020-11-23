function [v] = pnc(filename,velIng,k)

xInizio = 0;
xFine = 2.74;
xRete = 1.37;

yInizio = -0.76;
yFine = 0.76;

A=xlsread(filename);
Pos = A(:,1:3);
%Pos = [0, -0.32, 0.28];
velRacc = A(:,4:6);
target = A(1,9:11);
alfa = deg2rad(A(:,8));
numeroCampioni = length(A(:,1));
%k = [1 8 15];
posConsiderate = length(k);


%Disegno il campo

delta = 0.02;

fill([xInizio xInizio xFine xFine],[yInizio yFine yFine yInizio],'c');
line([xInizio xFine],[0 0],'LineWidth',1.5,'Color','White');
line([xRete xRete],[yInizio yFine],'LineWidth',1.5,'Color','White');
line([xInizio xFine],[yInizio yInizio],'LineWidth',3.5,'Color','White');
line([xInizio xFine],[yFine yFine],'LineWidth',3.5,'Color','White');
line([xInizio xInizio],[yInizio yFine],'LineWidth',3.5,'Color','White');
line([xFine xFine],[yInizio yFine],'LineWidth',3.5,'Color','White');
line([xInizio xFine],[yInizio yInizio],'LineWidth',0.4,'Color','Cyan');
line([xInizio xFine],[yFine yFine],'LineWidth',0.4,'Color','Cyan');
line([xInizio xInizio],[yInizio yFine],'LineWidth',0.4,'Color','Cyan');
line([xFine xFine],[yInizio yFine],'LineWidth',0.4,'Color','Cyan');

hold on
 w = 0.15;
 w1 = 0.25;
 modSeg = 1;
for i = 1 : posConsiderate
    px = Pos(k(i),1);
    py = Pos(k(i),2);
    vox = velRacc(k(i),1);
    voy = velRacc(k(i),2);
    azimutRa = alfa(k(i));
    x0 = plot(px,py,'ro');
    hold on
    x1 = plot([px target(1)],[py target(2)],'k>-.');
    hold on
    x2 = plot([px w1*vox+px],[py w1*voy+py],'b>-');
    hold on
    x3 = plot([px px-velIng(1)*w],[py py-velIng(2)*w],'m+-');
    hold on
    angOrt = azimutRa + pi/2*(-sign(azimutRa));
    x4 = plot([px px+modSeg*cos(angOrt)],[py py+ modSeg*sin(angOrt)],'r--');
    hold on
    x5 = plot([px-0.075*cos(azimutRa) px+0.075*cos(azimutRa)],[py-0.075*sin(azimutRa) py+0.075*sin(azimutRa)],'b-','LineWidth',2);
    hold on
end
x6 = plot(A(1,9),A(1,10),'r*');
legend([x0 x1 x2 x3 x4 x5 x6],'Impact point','Outgoing trajectory','Racket speed direction','Direction of v_{0}','Normal of racket surface','Racket','Target')

camroll(90)
axis([-0.1 2.9 -1.5 1.5]);
title('Rebound Simulation')
xlabel('x[m]');
ylabel('y[m]');
grid on

v = -1;
