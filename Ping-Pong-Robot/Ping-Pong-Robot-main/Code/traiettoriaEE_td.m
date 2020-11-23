function [posInit, posFin, tc, acc, soglia] = traiettoriaEE_td(platformB, tf , v)
%Pianificazione della traiettoria desiderata
%Gli input sono:
% - La posizione dell'End-Effector al momento dell'impatto (platformB)
% - Il tempo di durata del colpo (tf)
% - La velocità desiderata all'impatto (v)
acc = zeros(3,1);
deltaTc = zeros(3,1);
delta1Tc = zeros(3,1);
soglia = zeros(3,1);
posFin = zeros(3,1);
posInit = zeros(3,1);
vTf = zeros(3,1);
tc = tf / 3;
    for i = 1 : 3
        acc(i) = v(i) / tc;
        deltaTc(i) = 0.5*acc(i)*tc^2;
        delta1Tc(i) = v(i)*(tf - 2*tc)*0.5;
        posInit(i) = platformB(i) - deltaTc(i) - delta1Tc(i);
        posFin(i) = platformB(i) + deltaTc(i) + delta1Tc(i);
        vTf(i) = v(i) - acc(i)*tc;
        soglia(i) = deltaTc(i) + delta1Tc(i);
    end
end