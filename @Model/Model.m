classdef Model < handle & matlab.mixin.Copyable
 
    properties
        
        stDim = 4; % state dimension
        nEvents = 1; % num. of simulation events
        IC;
        
        % System parameters:
        
        m1 = 1;        % mass of link 1
        m2 = 1;        % mass of link 2
        l1 = 1;        % length of link 1
        l2 = 1;        % length of link 2
        lc1 = 0.5;     % center of mass of link 1
        lc2 = 0.5;     % center of mass of link 2
        I1 = m1*l1^2/12;    % moment of inertia of link 1
        I2 = m2*l2^2/12;    % moment of inertia of link 2
        g = 9.81;      %  gravity
        
        % Control:
        Torque = 0;
               
        % Render parameters:
        joint_radius = 0.08;
        joint_color = [0.2,0.6,0.8];
        
        link_width=0.025;
        link_color=[0.1, 0.3, 0.8];

        RenderObj;
        RenderVectors=0;
        RenderStance=0;
        RenderParams=0;
        
        CircRes=12;
        LinkRes=10;
        LineWidth=1;
        
        x0;
        y0;
        
    end
    
    methods
        %  Class constructor:
        function Mod = Model(varargin)
             % do nothing
        end

        
        %  Get position:
        function [ x, y ] = GetPos(Mod, q, which)
            
            theta1 = q(1);
            theta2 = q(3);
            
            if strcmp(which,'end1')
                x = Mod.l1*sin(theta1);
                y = -Mod.l1*cos(theta1);
                return;
            end
            
            if strcmp(which,'end2')
                
                [x1,y1] = Mod.GetPos(q, 'end1');
                
                x = x1+Mod.l2*sin(theta2);
                y = y1-Mod.l2*cos(theta2);
                return;
            end
            
            if strcmp(which,'COM')
                
                x = 0;
                y = 0;
                
            end
        end
        
        % Get velocity:
        function [ xdot, ydot ] = GetVel(Mod, q, which)

            theta1 = q(1);
            dtheta1 = q(2);
            theta2 = q(3);
            dtheta2 = q(4);

            if strcmp(which,'COM')
                xdot = Mod.L*cos(theta)*dtheta;
                ydot = Mod.L*sin(theta)*dtheta;
            end
        end
        
            
        % Derivative:
        function [Xdot] = Derivative(Mod, t, q) %#ok<INUSL>

            Xdot(1) =  q(2);
            Xdot(2) = -(Mod.m*Mod.g*Mod.L_cg*sin(q(1))+Mod.damping*q(2))/Mod.I + Mod.Torque;
            Xdot(3) =  q(4);
            Xdot(4) = -(Mod.m*Mod.g*Mod.L_cg*sin(q(1))+Mod.damping*q(2))/Mod.I + Mod.Torque;     
            
            Xdot = Xdot';
        end
        

        
        % Events:
        function [value, isterminal, direction] = Events(Mod, t,q, Floor)
            
            value = ones(Mod.nEvents,1);
            isterminal = ones(Mod.nEvents,1);
            direction = ones(Mod.nEvents,1);
            
            % Event #1 - end point is above goal height:
            value(1) = q(1)-Mod.SwitchAngle;
            isterminal(1) = 1;
            direction(1) = -1;
            
            
        end
        
        % Handle Events:
        function [Mod,qa] = HandleEvent(Mod, evID, qb, t)
            
           qa = qb;
            
            
            switch evID
                
                case 1
                  

                    
            end
        end
        

    end % end methods
end