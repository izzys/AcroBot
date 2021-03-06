classdef Controller < handle & matlab.mixin.Copyable

    
    properties
        
        stDim = 0; % state dimension of controller
        nEvents = 0; % num. of controller events
        IC;
        
        % Controller type:
        Controller_Type; 
        
        % HC (Heuristic control) parameters:
        InitialPotentialNrg;
        alpha = pi/7;
        h = 0.7;
        Kp = 5;
        Kd = 0;
        ControlON;
        

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
                
                Con.ControlON = 1;
                                               
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
                    
                 Con.ControlON = 1;
                 Xafter = Con.alpha;
                 
                 
                case 3
                    
                 Con.ControlON = 1;
                 Xafter = -Con.alpha;   
                     
                case 4
                    
                  Con.ControlON = 0;
                    
                otherwise
                  
                    return;
                  
                                      
            end
            
        end
        
        function [T] = Get_Torque(Con,X,Mod,t)
            
           switch Con.Controller_Type
               
               case 'off' 
                  
                   T = 0;      

               case 'HC'
                   
                   if Con.ControlON
                       
                       dq1 = X(2);
                       q2 = X(3);
                       dq2 = X(4);
                       q2_desired = X(5);

                       e = (q2 - q2_desired);
                       de = dq2;

                       [E] = Mod.GetNrg(X,'potential');
                       Efactor = E-Con.InitialPotentialNrg;

                       HeightFactor = Con.h*(1+Efactor);

                       T = -Con.Kp*e/HeightFactor-Con.Kd*de;

              %         T = -sign(dq1);
              
                  %   T = -sign(e);
                   else
                       
                       T=0;
                   end
               otherwise
                  
                    return;
                   
  
           end
           
        end

    end
end