function [yf]=risoluzioneMovimento(massa,g,step,t_fine,initial)
b=17.4e-06 ; %coefficiente di attrito viscoso dell'aria
tdelta =0:step:t_fine; %intervallo di risoluzione della prima eq cardinale
[t,yf] = ode45(@(t,Y) odefcn1_eq(t, Y, zeros(3,1), massa, g, b) , tdelta , initial);
end