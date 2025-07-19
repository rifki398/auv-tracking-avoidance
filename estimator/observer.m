function [alpha_c_hat, beta_c_hat, ehat, zetahat,e_tilde] = observer(e,pii,x,ehat,zetahat,Uvhat,Ts)
    nu = x(1:6);
    eta = x(7:12);

    pi_h = pii(1);
    pi_v = pii(2);

    theta = eta(5);
    psi = eta(6);

    u = nu(1);
    w = nu(3);

    Uv = sqrt(u^2 + w^2);

    [zetahat, ehat] = estimating(e,ehat,zetahat,pi_v,Ts);

    beta_c_hat = atan2(zetahat(2)/zetahat(1) - tan(psi-pi_h), 1 + (zetahat(2)/zetahat(1))*tan(psi-pi_h));

    if Uv == 0
        alpha_c_hat = 0;
    else
        val = clip(zetahat(3)/Uv,-1.0,1.0);
        alpha_c_hat = theta - asin(val);
    end
    e_tilde = ehat - e;
end

function [zetahat, ehat] = estimating(e,ehat,zetahat,pi_v,Ts)
    e_tilde = ehat - e;

    lambda1 = 60.5;
    lambda2 = 20.5;
    lambda3 = 80;

    dxe = 0.01; dye = 0.01; dze = 0.;

    zetahat_dot = [-lambda1*e_tilde(1); 
                -lambda2*e_tilde(2); 
                lambda3*e_tilde(3)];

    zetahat = zetahat_dot * Ts;

    ehat_dot = [zetahat(1)*cos(pi_v) + zetahat(3)*sin(pi_v) - sign(e_tilde(1))*dxe;
                zetahat(2) - sign(e_tilde(2))*dye;
                zetahat(1)*sin(pi_v) - zetahat(3)*cos(pi_v) - sign(e_tilde(3))*dze] ;

    ehat = ehat + ehat_dot * Ts;
end