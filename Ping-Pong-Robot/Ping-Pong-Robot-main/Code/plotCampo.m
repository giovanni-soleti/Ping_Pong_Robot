function [v] = plotCampo(sim)
% Plot del campo
xInizio = 0;
xFine = 2.74;
xRete = 1.37;
yInizio = -0.76;
yFine = 0.76;
zRete = 0.1725;

fill([xInizio xInizio xFine xFine],[yInizio yFine yFine yInizio],'c');
line([xInizio xFine],[0 0],'LineWidth',2,'Color','White');
line([xRete xRete],[yInizio yFine],'LineWidth',1.5,'Color','White');
line([xInizio xFine],[yInizio yInizio],'LineWidth',3.5,'Color','White');
line([xInizio xFine],[yFine yFine],'LineWidth',3.5,'Color','White');
line([xInizio xInizio],[yInizio yFine],'LineWidth',3.5,'Color','White');
line([xFine xFine],[yInizio yFine],'LineWidth',3.5,'Color','White');
line([xInizio xFine],[yInizio yInizio],'LineWidth',0.4,'Color','Cyan');
line([xInizio xFine],[yFine yFine],'LineWidth',0.4,'Color','Cyan');
line([xInizio xInizio],[yInizio yFine],'LineWidth',0.4,'Color','Cyan');
line([xFine xFine],[yInizio yFine],'LineWidth',0.4,'Color','Cyan');
fill3([xRete xRete xRete xRete],[yInizio yInizio yFine yFine],[0 zRete zRete 0],'b');
line([xRete xRete xRete xRete],[yInizio yInizio yFine yFine],[0 zRete zRete 0],'LineWidth',2.5,'Color','Blue');
line([xRete xRete xRete xRete],[yInizio yInizio yFine yFine],[0 zRete zRete 0],'LineWidth',1.5,'Color','White');

%plot target
plot3(sim(i,9),sim(i,10),sim(i,11),'ro');
%plot palla
plot3(sim(i,1),sim(i,2),sim(i,3),'o','MarkerFaceColor','White','MarkerSize',6);
v = -1;