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
        I1 = 1/12;     % moment of inertia of link 1
        I2 = 1/12;     % moment of inertia of link 2
        g = 9.81;      % gravity
        
        % Control:
        Torque = 0;
        GoalHeight = 1;
        
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
        
        x0=0;
        y0=0;
        
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
            
            if strcmp(which,'center1')
                x = Mod.lc1*sin(theta1);
                y = -Mod.lc1*cos(theta1);
                return;
            end
            
            if strcmp(which,'end2')
                
                [x1,y1] = Mod.GetPos(q, 'end1');
                
                x = x1+Mod.l2*sin(theta1+theta2);
                y = y1-Mod.l2*cos(theta1+theta2);
                return;
            end
            
            if strcmp(which,'center2')
                
                [x1,y1] = Mod.GetPos(q, 'end1');
                
                x = x1+Mod.lc2*sin(theta1+theta2);
                y = y1-Mod.lc2*cos(theta1+theta2);
                return;
            end
            
            if strcmp(which,'COM')
               
                [xc1,yc1] = Mod.GetPos(q, 'center1');
                [xc2,yc2] = Mod.GetPos(q, 'center2');
                
                x = (Mod.m1*xc1+Mod.m2*xc2)/(Mod.m1+Mod.m2);
                y = (Mod.m1*yc1+Mod.m2*yc2)/(Mod.m1+Mod.m2);
                
            end
        end
        
        % Get velocity:
        function [ xdot, ydot ] = GetVel(Mod, q, which)

            theta1 = q(1);
            dtheta1 = q(2);
            theta2 = q(3);
            dtheta2 = q(4);
            
            if strcmp(which,'end1')
                xdot = Mod.l1*cos(theta1)*dtheta1;
                ydot = Mod.l1*sin(theta1)*dtheta1;
                return;
            end           
            
            if strcmp(which,'center1')
                xdot = Mod.lc1*cos(theta1)*dtheta1;
                ydot = Mod.lc1*sin(theta1)*dtheta1;
                return;
            end
            
            if strcmp(which,'end2')
                
                [xdot1,ydot1] = Mod.GetVel(q, 'end1');
                
                xdot = xdot1+Mod.l2*cos(theta1+theta2)*(dtheta1+dtheta2);
                ydot = ydot1+Mod.l2*sin(theta1+theta2)*(dtheta1+dtheta2);
                return;
            end
            
            if strcmp(which,'center2')
                
                [xdot1,ydot1] = Mod.GetVel(q, 'end1');
                
                xdot = xdot1+Mod.lc2*cos(theta1+theta2)*(dtheta1+dtheta2);
                ydot = ydot1+Mod.lc2*sin(theta1+theta2)*(dtheta1+dtheta2);
                return;
            end
            
            if strcmp(which,'COM')
               
                [xdot_c1,ydot_c1] = Mod.GetVel(q, 'center1');
                [xdot_c2,ydot_c2] = Mod.GetVel(q, 'center2');
                
                xdot = (Mod.m1*xdot_c1+Mod.m2*xdot_c2)/(Mod.m1+Mod.m2);
                ydot = (Mod.m1*ydot_c1+Mod.m2*ydot_c2)/(Mod.m1+Mod.m2);
                
            end
        end
        
            
        % Derivative:
        function [qdot] = Derivative(Mod, t, q) %#ok<INUSL>

            m1 = Mod.m1;
            m2 = Mod.m2;
            l1 = Mod.l1;
            l2 = Mod.l2;
            lc1 = Mod.lc1;
            lc2 = Mod.lc2;
            I1 = Mod.I1;
            I2 = Mod.I2;
            g = Mod.g;
            
            theta1 = q(1); 
            dtheta1 = q(2);
            theta2 = q(3);
            dtheta2 = q(4);
            
            d1 = m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2*cos(theta2))+I1+I2;
            d2 = m2*(lc2^2+l1*lc2*cos(theta2))+I2;
            f2 = m2*lc2*g*cos(theta1+theta2-pi/2);
            f1 = -m2*l1*lc2*dtheta2^2*sin(theta2)-2*m2*l1*lc2*dtheta2*dtheta1*sin(theta2)...
                +(m1*lc1+m2*l1)*g*cos(theta1-pi/2)+f2;
            
            tau = Mod.Torque;
            
            ddtheta2 = (m2*lc2+I2-d2^2/d1)^(-1)*(tau+d2/d1*f1-m2*l1*lc2*dtheta1^2*sin(theta2)-f2);
            ddtheta1 = -d1^(-1)*(d2*ddtheta2+f1);
            
            qdot(1) =  q(2);
            qdot(2) =  ddtheta1;
            qdot(3) =  q(4);
            qdot(4) =  ddtheta2;
            
            qdot = qdot';
        end
        

        
        % Events:
        function [value, isterminal, direction] = Events(Mod, t,q, Floor)
            
            value = ones(Mod.nEvents,1);
            isterminal = ones(Mod.nEvents,1);
            direction = ones(Mod.nEvents,1);
            
            % Event #1 - end point is above goal height:
            [~,y] = Mod.GetPos(q,'end2');
            
            value(1) = y-Mod.GoalHeight;
            isterminal(1) = 1;
            direction(1) = -1;
            
            
        end
        
        % Handle Events:
        function [Mod,qa] = HandleEvent(Mod, evID, qb, t)
            
           qa = qb;
            
            
            switch evID
                
                case 1
                  
                otherwise
                    
                   return;
                    
            end
        end
        

    end % end methods
end