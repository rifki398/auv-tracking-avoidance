function [alpha_hat,beta_hat] = hybrid_observer(e,alpha_hat,beta_hat)
    Delta_h = 20;               % Horizontal look-ahead distance (m)
    Delta_v = 20;               % Vertical look-ahead distance (m)
    gamma_h = 0.001;            % Adaptive gain, horizontal plane
    gamma_v = 0.001;            % Adaptive gain, vertical plane
    M_theta = deg2rad(20);      % Maximum value of estimates, alpha_c, beta_c

    y_e = e(2);
    z_e = e(3);

    alpha_hat = alpha_hat + h * gamma_v * Delta_v / sqrt( Delta_v^2 + z_e^2 ) * proj(alpha_hat, z_e, M_theta);
    beta_hat = beta_hat + h * gamma_h * Delta_h / sqrt( Delta_h^2 + y_e^2 ) * proj(beta_hat, y_e, M_theta);
end

function y = proj(theta_hat, tau, M_theta)    
    eps = 0.001;
    M_theta_hat = M_theta + eps;

    if ( abs(theta_hat) > M_theta ) && ( theta_hat * tau > 0 )
        c = min(1, ( M_theta_hat^2 - M_theta^2) / (M_theta_hat^2 - M_theta^2) );
        y = (1 - c) * tau;
    else
        y = tau;
    end
end