classdef CandidateLeftTurn
    % candidate individual for left turn maneuver
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
      DEBUG=1;
    end
   properties
      geno
      fit
   end
   methods
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Constructor
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       function obj = CandidateLeftTurn(v)
           % Genotype
           if nargin == 1
            obj.geno = v;
           else
            % In general, you can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1).
            obj.geno = [
                round(CandidateLeftTurn.DELTA_MIN + (CandidateLeftTurn.DELTA_MAX-CandidateLeftTurn.DELTA_MIN).*rand(1,1)), % delta_l
                round(CandidateLeftTurn.DELTA_MIN + (CandidateLeftTurn.DELTA_MAX-CandidateLeftTurn.DELTA_MIN).*rand(1,1)), % delta_r
                round(CandidateLeftTurn.OMEGA_MIN + (CandidateLeftTurn.OMEGA_MAX-CandidateLeftTurn.OMEGA_MIN).*rand(1,1)), % omega_l
                round(CandidateLeftTurn.OMEGA_MIN + (CandidateLeftTurn.OMEGA_MAX-CandidateLeftTurn.OMEGA_MIN).*rand(1,1)), % omega_r
                round(CandidateLeftTurn.SIGMA_MIN + (CandidateLeftTurn.SIGMA_MAX-CandidateLeftTurn.SIGMA_MIN).*rand(1,1)), % sigma_1
                round(CandidateLeftTurn.SIGMA_MIN + (CandidateLeftTurn.SIGMA_MAX-CandidateLeftTurn.SIGMA_MIN).*rand(1,1)), % sigma_2
                round(CandidateLeftTurn.SIGMA_MIN + (CandidateLeftTurn.SIGMA_MAX-CandidateLeftTurn.SIGMA_MIN).*rand(1,1)), % sigma_3
                round(CandidateLeftTurn.SIGMA_MIN + (CandidateLeftTurn.SIGMA_MAX-CandidateLeftTurn.SIGMA_MIN).*rand(1,1)), % sigma_4
                ];
           end
           
           % Fitness
           obj.fit = ObjFit(obj);
       end
       
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Calculate Fitness
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %  F_l = omega_L*(-1*delta_L) + omega_R*delta_R
        function ffit = ObjFit(obj)
           ffit = obj.geno(CandidateLeftTurn.omega_L)*(-1*obj.geno(CandidateLeftTurn.delta_L)) + ...
                  obj.geno(CandidateLeftTurn.omega_R)*obj.geno(CandidateLeftTurn.delta_R);
        end
        
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
       % Mutate
       % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % return new genotype and new fitness
       function [n_geno, n_fit] = ObjMutate(obj)
          n_geno = obj.geno;
          % x_k = m_k + sigma_k*N(0,C) 
          % omega_l
          n_geno(CandidateLeftTurn.omega_L) = obj.geno(CandidateLeftTurn.omega_L) + ...
              round(obj.geno(CandidateLeftTurn.omega_L+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateLeftTurn.omega_L) > CandidateLeftTurn.OMEGA_MAX)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_L > OMEGA_MAX; omega_L=', num2str(n_geno(CandidateLeftTurn.omega_L)) ]);
                  end
              n_geno(CandidateLeftTurn.omega_L) = CandidateLeftTurn.OMEGA_MAX;
          else
              if (n_geno(CandidateLeftTurn.omega_L) < CandidateLeftTurn.OMEGA_MIN)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_L < OMEGA_MIN; omega_L=', num2str(n_geno(CandidateLeftTurn.omega_L)) ]);
                  end
               n_geno(CandidateLeftTurn.omega_L) = CandidateLeftTurn.OMEGA_MIN;
              else
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_L within bounds; omega_L=', num2str(n_geno(CandidateLeftTurn.omega_L)) ]);
                  end
              end
          end
          % omega_r
          n_geno(CandidateLeftTurn.omega_R) = obj.geno(CandidateLeftTurn.omega_R) + ...
              round(obj.geno(CandidateLeftTurn.omega_R+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateLeftTurn.omega_R) > CandidateLeftTurn.OMEGA_MAX)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_R > OMEGA_MAX; omega_R=', num2str(n_geno(CandidateLeftTurn.omega_R)) ]);
                  end
              n_geno(CandidateLeftTurn.omega_R) = CandidateLeftTurn.OMEGA_MAX;
          else
              if (n_geno(CandidateLeftTurn.omega_R) < CandidateLeftTurn.OMEGA_MIN)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_R < OMEGA_MIN; omega_R=', num2str(n_geno(CandidateLeftTurn.omega_R)) ]);
                  end
               n_geno(CandidateLeftTurn.omega_R) = CandidateLeftTurn.OMEGA_MIN;
              else
                 if (CandidateLeftTurn.DEBUG)
                    disp(['Omega_R within bounds; omega_R=', num2str(n_geno(CandidateLeftTurn.omega_R)) ]);
                  end 
              end
          end
          % delta_l
          n_geno(CandidateLeftTurn.delta_L) = obj.geno(CandidateLeftTurn.delta_L) + ...
              round(obj.geno(CandidateLeftTurn.delta_L+4)*(-1 + (2).*rand(1,1)));
          % bound
          if (n_geno(CandidateLeftTurn.delta_L) > CandidateLeftTurn.DELTA_MAX)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_L > DELTA_MAX; delta_L=', num2str(n_geno(CandidateLeftTurn.delta_L)) ]);
                  end
              n_geno(CandidateLeftTurn.delta_L) = CandidateLeftTurn.DELTA_MAX;
          else
              if (n_geno(CandidateLeftTurn.delta_L) < CandidateLeftTurn.DELTA_MIN)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_L < DELTA_MIN; delta_L=', num2str(n_geno(CandidateLeftTurn.delta_L)) ]);
                  end
               n_geno(CandidateLeftTurn.delta_L) = CandidateLeftTurn.DELTA_MIN;
              else
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_L within bounds; delta_L=', num2str(n_geno(CandidateLeftTurn.delta_L)) ]);
                  end
              end
          end
          % check delta < omega/2
          if (abs(n_geno(CandidateLeftTurn.delta_L)) >= n_geno(CandidateLeftTurn.omega_L)/2)
              if (CandidateLeftTurn.DEBUG)
                disp(['Delta_L >= omega_L/2; delta_L=', num2str(n_geno(CandidateLeftTurn.delta_L)) ]);
              end
              n_geno(CandidateLeftTurn.delta_L) = sign(n_geno(CandidateLeftTurn.delta_L))*floor(n_geno(CandidateLeftTurn.omega_L)/2);
          end
          % delta_r
          n_geno(CandidateLeftTurn.delta_R) = obj.geno(CandidateLeftTurn.delta_R) + ...
              round(obj.geno(CandidateLeftTurn.delta_R+4)*(-1 + (2).*rand(1,1)));
          if (n_geno(CandidateLeftTurn.delta_R) > CandidateLeftTurn.DELTA_MAX)
              if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_R > DELTA_MAX; delta_R=', num2str(n_geno(CandidateLeftTurn.delta_R)) ]);
              end
              n_geno(CandidateLeftTurn.delta_R) = CandidateLeftTurn.DELTA_MAX;
          else
              if (n_geno(CandidateLeftTurn.delta_R) < CandidateLeftTurn.DELTA_MIN)
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_R < DELTA_MIN; delta_R=', num2str(n_geno(CandidateLeftTurn.delta_R)) ]);
                  end
               n_geno(CandidateLeftTurn.delta_R) = CandidateLeftTurn.DELTA_MIN;
              else
                  if (CandidateLeftTurn.DEBUG)
                    disp(['Delta_R within bounds; delta_R=', num2str(n_geno(CandidateLeftTurn.delta_L)) ]);
                  end
              end
          end
          % check delta < omega/2
          if (abs(n_geno(CandidateLeftTurn.delta_R)) >= n_geno(CandidateLeftTurn.omega_R)/2)
              if (CandidateLeftTurn.DEBUG)
                disp(['Delta_R >= omega_R/2; delta_R=', num2str(n_geno(CandidateLeftTurn.delta_R)) ]);
              end
              n_geno(CandidateLeftTurn.delta_R) = sign(n_geno(CandidateLeftTurn.delta_R))*floor(n_geno(CandidateLeftTurn.omega_R)/2);
          end
          
          
          obj.geno = n_geno;
          n_fit = ObjFit(obj);
       end
   end
end