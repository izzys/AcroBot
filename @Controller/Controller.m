classdef Controller < handle & matlab.mixin.Copyable

    
    properties
        
        stDim = 1; % state dimension of controller
        nEvents = 1; % num. of controller events
        IC;
        
        % Controller type:
        Controller_Type; 
        
        
        % Pulse parameters:
        omega0;
        Phase = 0;
        Gain = 1;
        Period;
        Pulse_Width;

    end
    
    methods
        
        % Class constructor:
        function [NC] = Controller(varargin)
                    % do nothing
        end
        
        function [NC] = Init(NC)
            
          switch NC.Controller_Type
              
              case 'pulse'
                  
                NC.stDim = 1; 
                NC.nEvents = 1;
                                
              otherwise
                  
                  return;
           end    

        end

        function [Xdot] = Derivative(NC, t, X)
            
           switch NC.Controller_Type
              
              case 'pulse'
                  
                  Xdot = NC.omega0;
                               
              otherwise
                  
                  Xdot = 0;
           end    
   
        end
     
        function [value, isterminal, direction] = Events(NC,t, X) %#ok<INUSL>
            
            value = ones(NC.nEvents,1);
            isterminal = ones(NC.nEvents,1);
            direction = ones(NC.nEvents,1);
            
            switch NC.Controller_Type
                   
               case 'pulse'
                   
                   value = 2*pi-X;
                   isterminal = 1;
                   direction = -1;
                   
             
               otherwise
                  
                    return;
                   
             end
            
            
        end
        
        % Handle Events:
        function [NC,Xafter] = HandleEvent(NC, evID, Xbefore, t) %#ok<INUSD>
            
            Xafter = Xbefore;
            switch evID
                
                case 1 % end of periof
                  
                    Xafter = 0;
                    
                otherwise
                  
                    return;
                    
            end
            
        end
        
        function [NC,Xafter] = HandleExtEvent(NC, Ext_evID, Xbefore, t)

            Xafter = Xbefore(end-NC.stDim+1:end);
            
            switch Ext_evID
                
                case 1 
                    
                    % do nothing
                    
                otherwise
                  
                    return;
                  
                                      
            end
            
        end
        
        function [T] = Get_Torque(NC,X,t)
            
           switch NC.Controller_Type
               
               case 'off' 
                  
                   T = 0;      
                   
        
              
               case 'pulse'
 
               otherwise
                  
                    return;
                   
  
           end
           
        end

    end
end