function [Vc,alphaVc,betaVc,wc] = disturbance_function(t,h)
    Vc = 0.5;                      % Horizontal speed (m/s)
    betaVc = deg2rad(30);          % Horizontal direction (radians)
    alphaVc = deg2rad(10);
    wc = 0.1;                      % Vertical speed (m/s)
    if t > 300
        Vc_d = 0.65;
        w_V = 0.05;
        Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
    else
        Vc = 0.5;
    end

    if t > 500
        betaVc_d = deg2rad(160);
        w_beta = 0.1;
        betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
    else
        betaVc = deg2rad(150);
    end
    
    alphaVc = exp(-h*wc)*alphaVc + (1 - exp(-h*wc)) * alphaVc;

    alphaVc = alphaVc + randn / 500;
    betaVc = betaVc + randn / 1000;
    Vc = Vc + 0.002 * randn;