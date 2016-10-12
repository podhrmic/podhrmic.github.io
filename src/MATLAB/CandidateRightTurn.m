classdef CandidateRightTurn
    % candidate individual for right turn maneuver
    % genotype is {delta_l, delta_r, omega_l, omega_r,
    %              sigma_1, sigma_2, sigma_3, sigma_4}
    properties (Constant)
      OMEGA_MIN =1;
      OMEGA_MAX =30;
      DELTA_MIN = -10;
      DELTA_MAX = 10;
      SIGMA_MIN = 3;
      SIGMA_MAX = 6;
      text_geno = 'delta_l, delta_r, omega_l, omega_r,sigma_1, sigma_2, sigma_3, sigma_4'
      delta_L = 1;
      delta_R = 2;
      omega_L = 3;
      omega_R = 4;
      DEBUG=0;
      SIMULATION=0;
    end
   properties
      geno
      fit
   end
   methods
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Constructor
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       function obj = CandidateRightTurn(v)
           % Genotype
           if nargin == 1
            obj.geno = v;
           else
            % In general, you can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1).
            obj.geno = [
                round(CandidateRightTurn.DELTA_MIN + (CandidateRightTurn.DELTA_MAX-CandidateRightTurn.DELTA_MIN).*rand(1,1)), % delta_l
                round(CandidateRightTurn.DELTA_MIN + (CandidateRightTurn.DELTA_MAX-CandidateRightTurn.DELTA_MIN).*rand(1,1)), % delta_r
                round(CandidateRightTurn.OMEGA_MIN + (CandidateRightTurn.OMEGA_MAX-CandidateRightTurn.OMEGA_MIN).*rand(1,1)), % omega_l
                round(CandidateRightTurn.OMEGA_MIN + (CandidateRightTurn.OMEGA_MAX-CandidateRightTurn.OMEGA_MIN).*rand(1,1)), % omega_r
                round(CandidateRightTurn.SIGMA_MIN + (CandidateRightTurn.SIGMA_MAX-CandidateRightTurn.SIGMA_MIN).*rand(1,1)), % sigma_1
                round(CandidateRightTurn.SIGMA_MIN + (CandidateRightTurn.SIGMA_MAX-CandidateRightTurn.SIGMA_MIN).*rand(1,1)), % sigma_2
                round(CandidateRightTurn.SIGMA_MIN + (CandidateRightTurn.SIGMA_MAX-CandidateRightTurn.SIGMA_MIN).*rand(1,1)), % sigma_3
                round(CandidateRightTurn.SIGMA_MIN + (CandidateRightTurn.SIGMA_MAX-CandidateRightTurn.SIGMA_MIN).*rand(1,1)), % sigma_4
                ];
           end
           
           % Fitness
           obj.fit = ObjFit(obj);
       end
       
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Calculate Fitness
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %  F_l = omega_L*delta_L + omega_R*(-1*delta_R)
        function ffit = ObjFit(obj)
            if (CandidateRightTurn.SIMULATION)
                % simulated run
                ffit = obj.geno(CandidateRightTurn.omega_L)*obj.geno(CandidateRightTurn.delta_L) + ...
                       obj.geno(CandidateRightTurn.omega_R)*(-1*obj.geno(CandidateRightTurn.delta_R));                
            else
                disp('Insert time needed to turn right by 360deg.');
                disp(['Delta_L=', num2str(obj.geno(CandidateRightTurn.delta_L)),...
                    ', Delta_R=', num2str(obj.geno(CandidateRightTurn.delta_R)), ', Omega_L=', ...
                    num2str(obj.geno(CandidateRightTurn.omega_L)), ', Omega_R=', num2str(obj.geno(CandidateRightTurn.omega_R))]);
                time = input('Time=?[s]\n');
                ffit=1/time;                
            end
        end
        
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Mutate
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % return new genotype and new fitness
       function [n_geno, n_fit] = ObjMutate(obj)
          n_geno = obj.geno;
          % x_k = m_k + sigma_k*N(0,C) 
          % omega_l
          n_geno(CandidateRightTurn.omega_L) = obj.geno(CandidateRightTurn.omega_L) + ...
              round(obj.geno(CandidateRightTurn.omega_L+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateRightTurn.omega_L) > CandidateRightTurn.OMEGA_MAX)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Omega_L > OMEGA_MAX; omega_L=', num2str(n_geno(CandidateRightTurn.omega_L)) ]);
                  end
              n_geno(CandidateRightTurn.omega_L) = CandidateRightTurn.OMEGA_MAX;
          else
              if (n_geno(CandidateRightTurn.omega_L) < CandidateRightTurn.OMEGA_MIN)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Omega_L < OMEGA_MIN; omega_L=', num2str(n_geno(CandidateRightTurn.omega_L)) ]);
                  end
               n_geno(CandidateRightTurn.omega_L) = CandidateRightTurn.OMEGA_MIN;
              else
                  if (CandidateRightTurn.DEBUG)
                    disp(['Omega_L within bounds; omega_L=', num2str(n_geno(CandidateRightTurn.omega_L)) ]);
                  end
              end
          end
          % omega_r
          n_geno(CandidateRightTurn.omega_R) = obj.geno(CandidateRightTurn.omega_R) + ...
              round(obj.geno(CandidateRightTurn.omega_R+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateRightTurn.omega_R) > CandidateRightTurn.OMEGA_MAX)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Omega_R > OMEGA_MAX; omega_R=', num2str(n_geno(CandidateRightTurn.omega_R)) ]);
                  end
              n_geno(CandidateRightTurn.omega_R) = CandidateRightTurn.OMEGA_MAX;
          else
              if (n_geno(CandidateRightTurn.omega_R) < CandidateRightTurn.OMEGA_MIN)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Omega_R < OMEGA_MIN; omega_R=', num2str(n_geno(CandidateRightTurn.omega_R)) ]);
                  end
               n_geno(CandidateRightTurn.omega_R) = CandidateRightTurn.OMEGA_MIN;
              else
                 if (CandidateRightTurn.DEBUG)
                    disp(['Omega_R within bounds; omega_R=', num2str(n_geno(CandidateRightTurn.omega_R)) ]);
                  end 
              end
          end
          % delta_l
          n_geno(CandidateRightTurn.delta_L) = obj.geno(CandidateRightTurn.delta_L) + ...
              round(obj.geno(CandidateRightTurn.delta_L+4)*(-1 + (2).*rand(1,1)));
          % bound
          if (n_geno(CandidateRightTurn.delta_L) > CandidateRightTurn.DELTA_MAX)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Delta_L > DELTA_MAX; delta_L=', num2str(n_geno(CandidateRightTurn.delta_L)) ]);
                  end
              n_geno(CandidateRightTurn.delta_L) = CandidateRightTurn.DELTA_MAX;
          else
              if (n_geno(CandidateRightTurn.delta_L) < CandidateRightTurn.DELTA_MIN)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Delta_L < DELTA_MIN; delta_L=', num2str(n_geno(CandidateRightTurn.delta_L)) ]);
                  end
               n_geno(CandidateRightTurn.delta_L) = CandidateRightTurn.DELTA_MIN;
              else
                  if (CandidateRightTurn.DEBUG)
                    disp(['Delta_L within bounds; delta_L=', num2str(n_geno(CandidateRightTurn.delta_L)) ]);
                  end
              end
          end
          % check delta < omega/2
          if (abs(n_geno(CandidateRightTurn.delta_L)) >= n_geno(CandidateRightTurn.omega_L)/2)
              if (CandidateRightTurn.DEBUG)
                disp(['Delta_L >= omega_L/2; delta_L=', num2str(n_geno(CandidateRightTurn.delta_L)) ]);
              end
              n_geno(CandidateRightTurn.delta_L) = sign(n_geno(CandidateRightTurn.delta_L))*floor(n_geno(CandidateRightTurn.omega_L)/2);
          end
          % delta_r
          n_geno(CandidateRightTurn.delta_R) = obj.geno(CandidateRightTurn.delta_R) + ...
              round(obj.geno(CandidateRightTurn.delta_R+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateRightTurn.delta_R) > CandidateRightTurn.DELTA_MAX)
              if (CandidateRightTurn.DEBUG)
                    disp(['Delta_R > DELTA_MAX; delta_R=', num2str(n_geno(CandidateRightTurn.delta_R)) ]);
              end
              n_geno(CandidateRightTurn.delta_R) = CandidateRightTurn.DELTA_MAX;
          else
              if (n_geno(CandidateRightTurn.delta_R) < CandidateRightTurn.DELTA_MIN)
                  if (CandidateRightTurn.DEBUG)
                    disp(['Delta_R < DELTA_MIN; delta_R=', num2str(n_geno(CandidateRightTurn.delta_R)) ]);
                  end
               n_geno(CandidateRightTurn.delta_R) = CandidateRightTurn.DELTA_MIN;
              else
                  if (CandidateRightTurn.DEBUG)
                    disp(['Delta_R within bounds; delta_R=', num2str(n_geno(CandidateRightTurn.delta_L)) ]);
                  end
              end
          end
          % check delta < omega/2
          if (abs(n_geno(CandidateRightTurn.delta_R)) >= n_geno(CandidateRightTurn.omega_R)/2)
              if (CandidateRightTurn.DEBUG)
                disp(['Delta_R >= omega_R/2; delta_R=', num2str(n_geno(CandidateRightTurn.delta_R)) ]);
              end
              n_geno(CandidateRightTurn.delta_R) = sign(n_geno(CandidateRightTurn.delta_R))*floor(n_geno(CandidateRightTurn.omega_R)/2);
          end
          
          
          obj.geno = n_geno;
          n_fit = ObjFit(obj);
       end
   end
end