function dYdt = odefcn1_eq(t, Y, F, m, g, b)
%dove F è la forza applicata, m è la massa del corpo, g la costante di
%gravità e b il coefficiente di attrito viscoso, 0.47 coefficente resistivo
%dell'aria (Cx), 0.0025 superfice ortogonale sfera, 1.225 densità aria
vel=[Y(4);Y(5);Y(6)];
% modVel=norm(vel);
% alphaVel=atan(abs(Y(6)/Y(4)));
% velPortanza=[modVel*cos(alphaVel+pi/2);Y(5);modVel*sin(alphaVel+pi/2)];
% versPortanza=[Y(6);Y(5);-Y(4)]./norm(vel);
% dot(vel, versPortanza)
output=(1/m).*((-b*vel)+(F-m.*[0;0;g])); % (0.5*0.47*0.0025*1.225*(norm(vel)^2).*versPortanza)
Y(1)=output(1,1);
Y(2)=output(2,1);
Y(3)=output(3,1);
pos=[Y(1);Y(2);Y(3)];
dYdt = [vel;pos];
end