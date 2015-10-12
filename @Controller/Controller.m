classdef Controller < handle & matlab.mixin.Copyable

    
    properties
        
        stDim = 1; % state dimension of controller
        nEvents = 1; % num. of controller events
        IC;
        
        % Controller type:
        Controller_Type; 
        
        % Oscillator's frequency:
        omega0;
        
        % Pulse parameters:
        Phase = 0;
        Gain = 1;
        Period;
        Pulse_Width;
            
        % reflex parameters:
        ReflexOn = 0;
        
        % Hopf ocsillatior parameters:
        gamma;
        mu;
        
        % Addaptive Hopf ocsillator parameters:
        NumOfNeurons;
        eta;
        tau;
        epsilon;
        Amp_teach;
        Omega_teach;
        Phi_teach;
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
                
              case 'reflex'
                  
                NC.ReflexOn = 0; 
                
              case 'Hopf_general'
                  
                NC.stDim = 2; 
                NC.nEvents = 0;  
                
              case 'Hopf_adaptive'
                  
                NC.stDim = 5*NC.NumOfNeurons; 
                NC.nEvents = 0;  
                 
              otherwise
                  
                  return;
           end    

        end

        function [Xdot] = Derivative(NC, t, X)
            
           switch NC.Controller_Type
              
              case 'pulse'
                  
                  Xdot = NC.omega0;
                  
               case 'reflex'
                   
                  Xdot = 1;  
                  
               case 'Hopf_general'
                   
                   g = NC.gamma;
                   m = NC.mu;
                   w = NC.omega0;
                   x = X(1);
                   y = X(2);
                   r = sqrt(x^2+y^2);                 
                   
                   x_dot = g*(m-r^2)*x-w*y;
                   y_dot = g*(m-r^2)*y+w*x;
               
                   
                   Xdot = [x_dot ; y_dot ];
                   
               case 'Hopf_adaptive'
                   
                   for i = 0:NC.NumOfNeurons-1

                       g = NC.gamma;
                       m = NC.mu;
                       e = NC.epsilon;
                       h = NC.eta;
                       tau = NC.tau;

                       x = X(5*i+1);
                       y = X(5*i+2);
                       w = X(5*i+3);         
                       a = X(5*i+4);
                       f = X(5*i+5);

                       r = sqrt(x^2+y^2);                 
                       q = sign(x)*acos(-y/r);

                       w0 = w;
                       q0 = q;

                       R=w/w0*q0;

                       P = NC.Amp_teach*sin(NC.Omega_teach*t+NC.Phi_teach);  % teach signal

                       Q = a*x;
                       F = P-Q;

                       x_dot = g*(m-r^2)*x-w*y+e*F+tau*sin(q-f);
                       y_dot = g*(m-r^2)*y+w*x; 
                       w_dot = -e*F*y/r;
                       a_dot = h*x*F;
                       f_dot = sin(R-q-f);


                       Xdot(5*i+1:5*i+5,1) = [x_dot ; y_dot ; w_dot ; a_dot ; f_dot ] ;      
                   end 
%                    r=X(1);
%                    r_dot = g*(m-r^2)*r;
%                    phi_dot = w;
%                    Xdot = [r_dot ; phi_dot ];                
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
                    
                case 2
                    
                    % ???
                    
            end
            
        end
        
        function [NC,Xafter] = HandleExtEvent(NC, Ext_evID, Xbefore, t)

            Xafter = Xbefore(end-NC.stDim+1:end);
            
            switch Ext_evID
                
                case 1 % angle of pendulum < -0.1
                  
                    
                     switch NC.Controller_Type

                          case 'reflex'

                            Xafter = 0;
                            NC.ReflexOn = 1;


                         otherwise

                            return;

                     end
                    
                            
                case 2
                    
                    % ???
                    
            end
            
        end
        
        function [T] = Get_Torque(NC,X,t)
            
           switch NC.Controller_Type
               
               case 'off' 
                  
                   T = 0;      
                   
               case 'sin'
                   
                   phase = NC.Phase;
                   freq = NC.omega0;
                   T = NC.Gain*sin(freq*t(end)+phase);
              
               case 'pulse'
 
                   if X(3)<2*pi*NC.Pulse_Width
                        T = NC.Gain;           
                   else
                        T=0;
                   end
                   
               case 'reflex'
                   
                   
                   if X(3)<NC.Pulse_Width  && NC.ReflexOn
                       T = NC.Gain;
                   else
                       T=0;  
                   end
                   
               case 'Hopf_general'
                   
                        T = X(3);
                        
               case 'Hopf_adaptive' 
                   
                       T =  NC.Gain*X(3);
           end
           
        end

    end
end