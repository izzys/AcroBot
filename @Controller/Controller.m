classdef Controller < handle & matlab.mixin.Copyable

    
    properties
        
        stDim = 0; % state dimension of controller
        nEvents = 0; % num. of controller events
        IC;
        
        % Controller type:
        Controller_Type; 
        
        % HC (Heuristic control) parameters:
        InitialPotentialNrg;
        alpha = pi/2;
        Kp = 1;
        Kd = 0;
        

    end
    
    methods
        
        % Class constructor:
        function [Con] = Controller(varargin)
                    % do nothing
        end
        
        function [Con] = Init(Con)
            
          switch Con.Controller_Type
              
              case 'off'
                  
                Con.stDim = 0; 
                Con.nEvents = 0;
                
              case 'HC'
                  
                Con.stDim = 1; 
                Con.nEvents = 0;
                                               
              otherwise
                  
                  return;
           end    

        end

        function [Xdot] = Derivative(Con, t, X)
            
           switch Con.Controller_Type
              
              case 'off'
                  
                  Xdot = [];
                  
              case 'HC'
                  
                  Xdot = 0;
                               
              otherwise
                  
                  Xdot = [];
           end    
   
        end
     
        function [value, isterminal, direction] = Events(Con,t, X) %#ok<INUSL>
            
            value = ones(Con.nEvents,1);
            isterminal = ones(Con.nEvents,1);
            direction = ones(Con.nEvents,1);
            
            switch Con.Controller_Type
                   
                    
               otherwise
                  
                    return;
                   
             end
            
            
        end
        
        % Handle Events:
        function [Con,Xafter] = HandleEvent(Con, evID, Xbefore, t) %#ok<INUSD>
            
            Xafter = Xbefore;
            switch evID
                
                case 1 
                  
                  % do nothing;
                    
                otherwise
                  
                    return;
                    
            end
            
        end
        
        function [Con,Xafter] = HandleExtEvent(Con, Ext_evID, Xbefore, t)

            Xafter = Xbefore(end-Con.stDim+1:end);
            
            switch Ext_evID
                
                case 1 
                    
                % do nothing
                    
                case 2
                    
                 Xafter = sign(Xbefore(1))*Con.alpha;
                    
                otherwise
                  
                    return;
                  
                                      
            end
            
        end
        
        function [T] = Get_Torque(Con,X,Mod,t)
            
           switch Con.Controller_Type
               
               case 'off' 
                  
                   T = 0;      

               case 'HC'
                   
                   dq1 = X(2);
                   q2 = X(3);
                   dq2 = X(4);
                   q2_desired = X(5);
                   
                   e = (q2 - q2_desired);
                   de = dq2;
                   
                   [E] = Mod.GetNrg(X,'potential');
                   Efactor = E-Con.InitialPotentialNrg;
                   
                   HeightFactor = 1;%0.7*(1+Efactor);
                   
                 %  T = -Con.Kp*e/HeightFactor-Con.Kd*de;
                   
                     T = -sign(dq1);
               otherwise
                  
                    return;
                   
  
           end
           
        end

    end
end