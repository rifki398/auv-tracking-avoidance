function [threat_dist,detected_obs,r_obs,ang_obs,intersect,toward,idx] = sonar3D(x,etadot,target,obs)

r_max = 50;            % Sonar range
alpha = deg2rad(50); % elevation
beta = deg2rad(150); % azimuth

theta = x(11);
psi = x(12);

pos = x(7:9);

detected_obs = [];
r_obs = 0;
ang_obs = [0 0];
intersect = false;
toward = false;

threat_dist = zeros(1,length(obs.r));

for i = 1:length(obs.r)
    pos_obs = obs.pos{i};
    radius_obs = obs.r{i};
    d = pos - pos_obs;
    pos_to_obs = norm(d);
    threat_dist(i) = 1/((pos_to_obs-radius_obs)^2);
    % if pos(2) > 160 && pos(2) < 160.2
    %     fprintf('\n pos: [%.1f,%.1f,%.1f] with r_obs: %.0f with d: %.1f pos obs: [%.0f %.0f %.0f]',...
    %         pos(1),pos(2),pos(3),radius_obs,pos_to_obs,pos_obs(1),pos_obs(2),pos_obs(3))
    % end

    if pos_to_obs <= radius_obs + r_max
        % fprintf('Obs detected in range %.2f from pos. [%.2f %.2f %.2f]\n',r_obs, x(7), x(8), x(9));
        % fprintf('Obs position at: %.2f',detected_obs)
        psi_s = atan2(pos_obs(2)-pos(2), pos_obs(1)-pos(1));
        r_s = norm(pos_obs(1:2) - pos(1:2));
        theta_s = atan2(-(pos_obs(3)-pos(3)),r_s);
        
        dpsi = wrapToPi(psi_s - psi);
        dtheta = wrapToPi(theta_s - theta);
        if abs(dpsi) <= beta/2 && abs(dtheta) <= alpha/2
            % Deteksi
            detected_obs = pos_obs;
            r_obs = pos_to_obs - radius_obs;
            ang_obs = [dpsi, dtheta];
        end
        % if psi + (beta/2) >= psi_s && psi_s - (beta/2) <= psi_s && theta + (alpha/2) >= theta_s ...
        %      && theta - (alpha/2) <= theta_s
        %     % If sonar detected something
        %     detected_obs = pos_obs;
        %     r_obs = pos_to_obs - radius_obs;
        %     ang_obs = [psi_s-psi, theta_s-theta];
        % end

        if line_intersects_sphere(pos,target,pos_obs,radius_obs,5)
            intersect = true;
        else
            intersect = false;
        end

        % check wether AUV move towards obstacle
        if toward_obstacle(etadot(1:3),pos,pos_obs,radius_obs)
            toward = true;
        else
            toward= false;
        end

    end

    if intersect && ~isempty(detected_obs)
        idx = i;
        return
    end
    idx = 0;
end

end

function state = line_intersects_sphere(pos,target,obs,radius,tol)
    d = target - pos;
    f = pos - obs;

    a = dot(d, d);
    b = 2 * dot(f, d);

    c = dot(f, f) - (radius+tol)^2;

    discriminant = b^2 - 4*a*c;
    state = discriminant >= 0;
end

function state = toward_obstacle(etadot,pos,obs,radius)
    if norm(etadot) == 0
        state = false;
        return
    end

    v1 = obs - pos;
    etadot_hat = etadot / norm(etadot);

    proj = dot(v1,etadot_hat);
    closest_point = pos + proj * etadot_hat;
    d = norm(closest_point - obs);

    if proj < 0
        state = false;
        return
    end
    state = d <= radius;

end
