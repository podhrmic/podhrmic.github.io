%% Evolution strategy test for delta and omega
% To find best delta/omega combination for turn left and turn right
% Author: Michal Podhradsky, michal.podhradsky@pdx.edu, 2016
% 
% (1,10)-ES
% genotype is {delta_l, delta_r, omega_l, omega_r,
%              sigma_1, sigma_2, sigma_3, sigma_4}
%
% The allowed values are limited by hardware:
% omega (1,30) [rad/s] which is roughly 4.7Hz max flapping frequency
% delta (-10,10) [rad/s] 
%
% Constraints: delta < omega/2 (otherwise the delta shift will be too fast
%                               for the hardware)
%
%
% Fitness: 
% in our mock situation, delta is the value and omega is multiplier
% and we are evaluation those for each maneuver (turn left and turn right)
%
% for example TURN_LEFT: F_l = omega_L*(-1*delta_L) + omega_R*delta_R
%
% that will give us a rought example and will follow the intuition 
% (higher delta and higher omega will give fastest response)
% Fitness is inversely proportional with the response time
% higher fitness -> lower response time

%% Clear
clear variables;
close all;
clc;
figure;
N_RUNS = 20;

%% Initial conditions
for m=1:N_RUNS

    disp(['Run #', num2str(m)]);
    
N_PARENTS = 1;
N_MUTANTS = 10;
N_GENERATIONS = 20;
DEBUG = 0;
itr=1;

% best solution in each generation
best_gen = nan(N_GENERATIONS,2);

% first parent
p1 = CandidateRightTurn;
best_so_far = p1;

for k=1:N_GENERATIONS
    if (DEBUG)
        disp(['Generations # ', num2str(k)]);
    end
    
    % Create offspring
    off(1:N_MUTANTS) = p1;
    
    % Mutate
    for l=1:N_MUTANTS
       if (DEBUG)
         disp(['Mutant # ', num2str(l)]);
       end
       [n_gen, n_fit] = off(l).ObjMutate;
       n_gen;
       n_fit;
       off(l).geno = n_gen;
       off(l).fit = n_fit;
       itr=itr+1;
    end
    
    % Select the best from the generation
    for l=1:N_MUTANTS
       if (off(l).fit > p1.fit)
          p1 = off(l); 
       end
    end
    
   % Save the best in the generation
   best_gen(k,1) = p1.fit;
    
    % Save the best so far
    if (p1.fit > best_so_far.fit)
       best_so_far = p1; 
       if (DEBUG)
         disp('Best so far:');
         best_so_far.geno
       end
    end
    
   % Save the best so far
   best_gen(k,2) = best_so_far.fit;
end


disp('Done!')
disp(['Total iterations: ', num2str(itr)])

if (DEBUG)
disp('Best so far:')
best_so_far.geno
end

% Visualize
%figure;
plot(1:N_GENERATIONS, best_gen(:,1),'b-','LineWidth', 2);
hold on;
grid on;
%plot(1:N_GENERATIONS, best_gen(:,2),'r-o');
xlabel('# generation')
ylabel('fitness')
%legend('best from generation', 'best so far')

end

%% Save
set(gca,'FontSize',20)
xlhand = get(gca,'xlabel');
set(xlhand,'string','# generations','fontsize',20) 
ylhand = get(gca,'ylabel');
set(ylhand,'string','fitness','fontsize',20)  
%tlhand = get(gca,'title');
%set(tlhand,'string','Turn left','fontsize',24)  
scrsz = get(0,'ScreenSize');
saveas(gcf, 'extrinsic.png')