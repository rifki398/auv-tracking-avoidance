classdef WOA_PathPlanning < WhaleOptimizationAlgorithm
    properties
        radius
        obstacles
        safe_distance = 5.0
        ngens = 20
    end
    
    methods
        %% Constructor
        function obj = WOA_PathPlanning(dim, opt_func, boundary, nsols, b, a, a_step, maximize)
            obj@WhaleOptimizationAlgorithm(dim, opt_func, boundary, nsols, b, a, a_step, maximize);
            obj.opt_func = @(x) obj.f(x); % pakai method f sebagai default cost
            
        end
        
        %% Cost functions
        function cost = f2(obj, x, xk, xk1, obs, r)
            % ... terjemahkan isi _f2 dari Python ke MATLAB
            disp(r)
            x = x(:)';  % pastikan row vector

            J_s = norm(x - xk);
            J_g = norm(x - xk1);

            D_obs = norm(x - obs);
            J_d = 1/(D_obs^2 + 1e-6);

            % Psi/theta
            v1 = x - xk;
            v2 = obs - xk;
            v3 = xk1 - xk;
            [psi, theta] = obj.angle_magnitude_between_two_vector(v1, v2);

            v4 = x - xk1;
            v5 = obs - xk1;
            [psi1, theta1] = obj.angle_magnitude_between_two_vector(v4, v5);

            if abs(psi1) > deg2rad(45) || abs(theta1) > deg2rad(30)
                J_angle_k1 = 1e6;
            else
                J_angle_k1 = 0;
            end

            J_angle_k = (psi - 0)^2 + (theta - 0)^2;
            J_angle = 5*J_angle_k + J_angle_k1;

            % Turn penalty
            [dir_h, dir_v] = obj.angle_direction_between_two_vector(v3, v2);
            [alt_dir_h, alt_dir_v] = obj.angle_direction_between_two_vector(v3, v1);
            side_h = obj.side_penalty(dir_h, alt_dir_h);
            side_v = obj.side_penalty(dir_v, alt_dir_v);
            J_side = (side_h + side_v) * 5;

            % Intersection penalty
            if obj.check_intsersect(xk, x, obs, r, obj.safe_distance) || obj.check_intsersect(x, xk1, obs, r, obj.safe_distance)
                penalty = 1e6;
            else
                penalty = 0;
            end

            % Weights
            w_s=0.25; w_g=0.25; w_d=0.11; w_angle=0.28;
            cost = w_s*J_s + w_g*J_g + w_d*J_d + w_angle*J_angle + J_side + penalty;
        end

        function cost = f(obj, x, xk, xk1, obs, r)
            x = x'; % pastikan row
            % v2 = xk1 - xk;
            % v2hat = v2 / norm(v2);

            D_obs = norm(x - obs);
            J_obs = 1/(D_obs^2 + 1e-6);
            J_s = norm(x - xk);
            J_g = norm(x - xk1);

            if obj.check_intsersect(xk, x, obs, r, obj.safe_distance) || ...
               obj.check_intsersect(x, xk1, obs, r, obj.safe_distance)
                penalty = 1e6;
            else
                penalty = 0;
            end

            w_obs=0.4; w_s=0.3; w_g=0.3;
            cost = w_obs*J_obs + w_s*J_s + w_g*J_g + penalty;
        end
        
        %% Helper methods
        function p = side_penalty(~, dir, alt_dir)
            if dir ~= alt_dir || dir==0
                p=0;
            else
                p=1;
            end
        end

        function [psi, theta] = angle_magnitude_between_two_vector(~, v1, v2)
            r1 = norm(v1(1:2));
            r2 = norm(v2(1:2));
            dz_v1 = v1(3); dz_v2 = v2(3);

            dot_psi = dot(v1(1:2), v2(1:2)) / (r1*r2+1e-6);
            psi = acos(dot_psi);

            dot_theta = dot([r1,dz_v1],[r2,dz_v2]) / (norm([r1,dz_v1])*norm([r2,dz_v2])+1e-6);
            theta = acos(dot_theta);
        end

        function [dir_h, dir_v] = angle_direction_between_two_vector(~, v1, v2)
            if norm(v1)==0 || norm(v2)==0
                dir_h=0; dir_v=0; return;
            end
            r1=norm(v1(1:2)); r2=norm(v2(1:2));
            dz_v1=v1(3); dz_v2=v2(3);

            cross_h = v1(1)*v2(2) - v1(2)*v2(1);
            dir_h = sign(cross_h);

            cross_v = r1*dz_v2 - dz_v1*r2;
            dir_v = sign(cross_v);
        end

        function res = check_intsersect(~, pos, target, obs, r, tol)
            d = target - pos;
            f = pos - obs;

            a = dot(d,d);
            b = 2*dot(f,d);
            c = dot(f,f)-(r+tol)^2;
            

            discriminant = b^2 - 4*a*c;
            res = discriminant >=0;
        end

        function res = toward_obstacle(~, etadot, pos, obs, r)
            if norm(etadot)==0
                res=false; return;
            end
            v1=obs-pos;
            etadot_hat=etadot/norm(etadot);
            proj=dot(v1,etadot_hat);
            closest_point=pos+proj*etadot_hat;
            d=norm(closest_point - obs);
            if proj<0
                res=false;
            else
                res=d<=r;
            end
        end

        %% Replan method
        function best_sol = repath_planning(obj, pos, goal, obs, r)
            obj.renew_cost_function(@(x)obj.f(x,pos,goal,obs,r));
            
            boundary_range=50;
            boundary = [obs(1)-boundary_range, obs(1)+boundary_range;
                        obs(2)-boundary_range, obs(2)+boundary_range;
                        0, obs(3)+boundary_range];
            obj.renew_init_sols(boundary);
            for i=1:obj.ngens
                obj.optimize();
            end
            [~,best_sol]=obj.get_best_solution();
        end

        %% Obstacle location estimation
        function obs_pos = obstacle_location(~, eta, r, ang)
            phi=eta(4); theta=eta(5); psi=eta(6);
            xs=eta(1); ys=eta(2); zs=eta(3);

            p_l = r*[cos(ang(1))*cos(ang(2)); sin(ang(1))*cos(ang(2)); sin(ang(2))];

            R = Rzyx(phi,theta,psi);
            obs = R*p_l + [xs;ys;zs];
            obs_pos=obs(:)';
        end
        
    end
end
