classdef Model < handle & matlab.mixin.Copyable

    
    properties
        
        
        stDim = 2; % state dimension
        nEvents = 1; % num. of simulation events
        IC;
        
        % System parameters:
        
        m = 2.8;        %  mass
        L = 1;          %  length
        L_cg = 0.5;     %  center of mass
        I = 0.411;      %  moment of inertia
        g = 9.81;       %  gravity
        damping = 0.05; %  damping
        
        % Control:
        Torque = 0;
        SwitchAngle = -0.1;
        
        
        % Set keys:
        SetKeys = {'m','L','L_cg','damping','I'};
                
        % Render parameters:
        m_radius = 0.08;
        m_color = [0.2,0.6,0.8];
        
        leg_width=0.025;
        leg_color=[0.1, 0.3, 0.8];

        
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
        function [ x, y ] = GetPos(Mod, X, which)
            
            theta = X(1);
            
            if strcmp(which,'m')
                x = Mod.L*sin(theta);
                y = -Mod.L*cos(theta);
                return;
            end

            if strcmp(which,'COM')
                
                x = 0;% Mod.L*sin(theta)/2;
                y = 0;%-Mod.L*cos(theta)/2;
                
            end
        end
        
        % Get velocity:
        function [ xdot, ydot ] = GetVel(Mod, X, which)

            theta = X(1);
            dtheta = X(2);
            
            if strcmp(which,'m')
                xdot = Mod.L*cos(theta)*dtheta;
                ydot = Mod.L*sin(theta)*dtheta;
                return;
            end

            if strcmp(which,'COM')
                xdot = Mod.L*cos(theta)*dtheta;
                ydot = Mod.L*sin(theta)*dtheta;
            end
        end
        
            
        % Derivative:
        function [Xdot] = Derivative(Mod, t, X) %#ok<INUSL>

            Xdot(1) = X(2);
            Xdot(2) = -(Mod.m*Mod.g*Mod.L_cg*sin(X(1))+Mod.damping*X(2))/Mod.I + Mod.Torque;
            
            Xdot = Xdot';
        end
        

        
        % Events:
        function [value, isterminal, direction] = Events(Mod, t,X, Floor)
            
            value = ones(Mod.nEvents,1);
            isterminal = ones(Mod.nEvents,1);
            direction = ones(Mod.nEvents,1);
            
            % Event #1 - pendulum passes -0.1 rad:
            value(1) = X(1)-Mod.SwitchAngle;
            isterminal(1) = 1;
            direction(1) = -1;
            
            
        end
        
        % Handle Events:
        function [Mod,Xa] = HandleEvent(Mod, evID, Xb, t)
            
            Xa = Xb;
            
            
            switch evID
                
                case 1
                  
                    % ???
                    
                case 2
                    
                    % ???
                    
            end
        end
        

    end % end methods
end