classdef WhaleOptimizationAlgorithm
    % Whale Optimization Algorithm (WOA) for local path planning in obstacle avoidance task
    properties
        dim                 % dimension of variable
        opt_func            % optimization function handle
        boundary            % [dim x 2] lower and upper bound
        nsols               % number of solutions
        maximize = false    % true if you want to maximize
        b
        a
        a_step
        best_solutions      % history of best solutions
        sols                % current solutions
    end
    
    methods
        %% Constructor
        function obj = WhaleOptimizationAlgorithm(dim, opt_func, boundary, nsols, b, a, a_step, maximize)
            if nargin > 0
                obj.dim = dim;
                obj.opt_func = opt_func;
                obj.boundary = boundary;
                obj.nsols = nsols;
                obj.b = b;
                obj.a = a;
                obj.a_step = a_step;
                if nargin >= 8
                    obj.maximize = maximize;
                end
                obj.best_solutions = {};
                obj.sols = [];
            end
        end
        
        %% Initialize or renew solutions
        function obj = renew_init_sols(obj, boundary)
            obj.boundary = boundary;
            obj.sols = obj.init_solutions();
        end
        
        %% Return current solutions
        function sols = get_solutions(obj)
            sols = obj.sols;
        end
        
        %% Optimize (one iteration)
        function obj = optimize(obj)
            if isempty(obj.sols)
                error('Initialize solutions first using renew_init_sols()');
            end
            
            ranked_sol = obj.rank_solutions();
            best_sol = ranked_sol(1,:);
            
            new_sols = best_sol;  % include best solution in new generation
            
            for i=2:size(ranked_sol,1)
                s = ranked_sol(i,:);
                
                if rand > 0.5
                    A = obj.compute_A();
                    if norm(A) < 1.0
                        new_s = obj.encircle(s, best_sol, A);
                    else
                        idx = randi(size(obj.sols,1));
                        rand_sol = obj.sols(idx,:);
                        new_s = obj.search(s, rand_sol, A);
                    end
                else
                    new_s = obj.attack(s, best_sol);
                end
                
                new_s = obj.boundary_solution(new_s);
                new_sols = [new_sols; new_s];
            end
            
            obj.sols = new_sols;
            obj.a = obj.a - obj.a_step;
        end
        
        %% Initialize solutions uniformly random
        function sols = init_solutions(obj)
            sols = zeros(obj.nsols, obj.dim);
            for d = 1:obj.dim
                lb = obj.boundary(d,1);
                ub = obj.boundary(d,2);
                sols(:,d) = lb + (ub - lb) .* rand(obj.nsols,1);
            end
        end
        
        %% Keep solution inside boundary
        function bounded = boundary_solution(obj, sol)
            bounded = sol;
            for d=1:obj.dim
                lb = obj.boundary(d,1);
                ub = obj.boundary(d,2);
                if bounded(d) < lb
                    bounded(d) = lb;
                elseif bounded(d) > ub
                    bounded(d) = ub;
                end
            end
        end
        
        %% Cost function wrapper
        function cost = cost_function(obj, x)
            cost = obj.opt_func(x);
        end
        
        %% Rank solutions
        function ranked = rank_solutions(obj)
            n = size(obj.sols,1);
            fitness = zeros(n,1);
            for i=1:n
                fitness(i) = obj.cost_function(obj.sols(i,:));
            end
            
            if obj.maximize
                [~, idx] = sort(fitness,'descend');
            else
                [~, idx] = sort(fitness,'ascend');
            end
            
            obj.best_solutions{end+1} = [fitness(idx(1)), obj.sols(idx(1),:)];
            ranked = obj.sols(idx,:);
        end
        
        %% Print best solution history
        function print_best_solutions(obj)
            fprintf('generation best solution history\n');
            disp(obj.best_solutions);
            
            best = obj.get_best_solution();
            fprintf('\nbest solution:\n');
            disp(best);
        end
        
        %% Get best solution
        function [fitness, value] = get_best_solution(obj)
            if isempty(obj.best_solutions)
                fitness = [];
                value = [];
                return;
            end
            all = cell2mat(obj.best_solutions');
            fitnesses = all(:,1);
            sol = all(:,2:end);
            
            if obj.maximize
                [fitness, idx] = max(fitnesses);
            else
                [fitness, idx] = min(fitnesses);
            end
            value = sol(idx,:);
        end
        
        %% Renew cost function
        function obj = renew_cost_function(obj,f)
            obj.opt_func = f;
        end
        
        %% Compute A and C
        function A = compute_A(obj)
            r = rand(1,obj.dim);
            A = 2*obj.a.*r - obj.a;
        end
        
        function C = compute_C(obj)
            C = 2*rand(1,obj.dim);
        end
        
        %% Encircle
        function new_s = encircle(obj, sol, best_sol, A)
            D = obj.encircle_D(sol, best_sol);
            new_s = best_sol - A .* D;
        end
        
        function D = encircle_D(obj, sol, best_sol)
            C = obj.compute_C();
            D = abs(C .* best_sol - sol);
        end
        
        %% Search
        function new_s = search(obj, sol, rand_sol, A)
            D = obj.search_D(sol, rand_sol);
            new_s = rand_sol - A .* D;
        end
        
        function D = search_D(obj, sol, rand_sol)
            C = obj.compute_C();
            D = abs(C .* rand_sol - sol);
        end
        
        %% Attack
        function new_s = attack(obj, sol, best_sol)
            D = abs(best_sol - sol);
            L = -1 + 2*rand(1,obj.dim); % uniform in [-1,1]
            new_s = D .* exp(obj.b * L) .* cos(2*pi*L) + best_sol;
        end
    end
end
