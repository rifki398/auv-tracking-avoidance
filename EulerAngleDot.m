function etadot = EulerAngleDot(x)

nu = x(1:6);
eta = x(7:12);

etadot = zeros(6);
p_dot   = Rzyx(eta(4), eta(5), eta(6)) * nu(1:3);
v_dot   = Tzyx(eta(4), eta(5)) * nu(4:6);

% Forward Euler integration
etadot(1:3) = p_dot;
etadot(4:6) = v_dot;

end