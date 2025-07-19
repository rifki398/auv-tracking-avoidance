function [Uvhat,Uv] = calculate_uhat(etadot,nu,beta_c_hat)
    Uh_hat = sqrt(etadot(1)^2 + etadot(2)^2);
    u_hat = Uh_hat * cos(beta_c_hat);

    Uv = sqrt(nu(1)^2 + nu(3)^2);
    Uvhat = sqrt(u_hat^2 + etadot(3)^2);