classdef Simulation < handle & matlab.mixin.Copyable
    
    properties
        Mod; % Model
        Con; % Controller
        Env; % Environment

        % State params
        stDim; ModCo; ConCo;
        % Event params
        nEvents; ModEv; ConEv;
    
        % Simulation parameters
        IC;
        infTime;
        tstep; tstep_normal; tstep_small = [];
        tstart; tend; tspan;
        
        % Performance tracking / Statistics
        Out; % output holder
        EndCond = 0;
        % Set EndCond to run the sim until:
        % 0 - the end of time
        % [1,numsteps] - numsteps are taken on end_slope
        % 2 - the system converges to a limit cycle
        
        StepsTaken;
        ICstore; nICsStored = 10;
        minDiff = 8e-7; % Min. difference for LC convergence
        stepsReq = 10; % Steps of minDiff required for convergence
        stepsSS; % Steps taken since minDiff
        
        % Poincare map calculation parameters
        IClimCyc; Period;
        PMeps = 1e-7; PMFull = 0;
        PMeigs; PMeigVs;
        % Check convergence progression
        doGoNoGo = 1; % 0 - OFF, 1 - Extend, 2 - Cut
        GNGThresh = [4,4]; % required steps for go/no-go order
        minMaxDiff = [1,0];
        ConvProgr = [0,0];
                
        % Rendering params
        Graphics = 1;
        Fig = 0; Once = 1; StopSim;
        FigWidth; FigHeight; AR;
        % Environment display
        FlMin; FlMax; HeightMin; HeightMax;
        
        % COM transformation
        tCOM; COMx0; COMy0;
        % Time display
        hTime; TimeStr = ['t = %.2f s\nSteps: %s '];
        % Convergence display
        hConv; ConvStr = 'Diff = %.2e\nPeriod = %s';

        Colors = {[1 0 0],[0 0 1],[0 1 0],[0 0 0]};
        
       
    end
    
    methods
        % Class constructor:
        function sim = Simulation(varargin)

                    sim.Mod = Model();
                    sim.Con = Controller();
                    sim.Env = Terrain();
           
        end
        
        % Make a deep copy of a handle object.
        function SimDC = deepcopy(sim)
            % Instantiate new object of the same class.
            SimDC = copy(sim);
            SimDC.Mod = copy(sim.Mod);
            SimDC.Con = copy(sim.Con);
            SimDC.Env = copy(sim.Env);
        end
        
        function sim = SetEndCond(sim, value)
            L = length(value);
            if L<1
                error('Invalid input for EndCond');
            end
            
            if value(1) == 1
                Error = ['When setting EndCond to 1,',...
                           'a value for num. steps is also needed',...
                           '\nPlease use sim.EndCond = [1,nsteps]'];
                if L<2
                    error(Error);
                else
                    if ~isnumeric(value(2)) || value(2)<1
                        error(Error);
                    end
                end
            end
            
            sim.EndCond = value;
        end
        
        function sim = SetTime(sim,tstart,tstep,tend)
            if nargin~=4
                error(['Set time expects 3 input arguments',...
                    ' but was provided with ',num2str(nargin)]);
            end
            sim.tstart = tstart;
            sim.tstep_normal = tstep;
         %   sim.tstep_small = tstep/3;
            sim.tstep = tstep;
            if isnumeric(tend)
                if tend<=tstart+tstep
                    error('tend is too close to tstart');
                else
                    sim.tend = tend;
                end
                sim.infTime = 0;
            else
                if strcmp(tend,'inf')
                    % Simulation will run for indefinite time
                    sim.infTime = 1;
                    sim.tend = 10;
                end
            end
            sim.Out.Tend = sim.tend;
        end
        
        function [Xt] = Derivative(sim,t,X)
            
            Xt = [sim.Mod.Derivative(t,X(sim.ModCo));
                  sim.Con.Derivative(t,X(sim.ConCo))];
        end

        function [value, isterminal, direction] = Events(sim, t, X) 
            
            value = zeros(sim.nEvents,1);
            isterminal = ones(sim.nEvents,1);
            direction = zeros(sim.nEvents,1);

            % Call model event function
            [value(sim.ModEv), isterminal(sim.ModEv), direction(sim.ModEv)] = ...
                sim.Mod.Events(t,X(sim.ModCo), sim.Env);
            
            % Call controller event function
            [value(sim.ConEv), isterminal(sim.ConEv), direction(sim.ConEv)] = ...
                sim.Con.Events(t,X(sim.ConCo));
            

        end
        
        function [status] = Output_function(sim,t,X,flag) 
           
           switch flag
               
               case 'init'
                  
                  % get controler action - first time:
                  sim.Mod.Torque = sim.Con.Get_Torque(sim.IC,t(1)); 
                  
                  % save for out:
                  sim.Out.Torque = [sim.Out.Torque ; sim.Mod.Torque];
                  sim.Out.Torque_time  = [sim.Out.Torque_time  t(1)];
                  
        
               case 'done'

                   return;
                            
               otherwise
                   
                  % get controler action:
                  sim.Mod.Torque = sim.Con.Get_Torque(X(:,end),t(end));
                  
                  % save for out:
                  sim.Out.Torque = [sim.Out.Torque ; sim.Mod.Torque];
                  sim.Out.Torque_time  = [sim.Out.Torque_time  t(end)];
                  
           end
           
           if sim.Graphics == 1
             sim.Render(t,X,flag);
           end
           
           status = sim.StopSim; 
            
        end

        function StopButtonCb(sim, hObject, eventdata, handles) %#ok<INUSD>
            if sim.StopSim == 0
                sim.StopSim = 1;
                sim.Out.Type = -1;
                sim.Out.Text = 'Simulation stopped by user';
                set(hObject,'String','Close Window');
            else
                close(sim.Fig)
            end
        end  
        
      
        
        function out = JoinOuts(sim,ext_out,last_i)
            if nargin<3
                last_i = length(sim.Out.T);
            end
            
            out = sim.Out;
            if isempty(ext_out) || length(ext_out.T)<1
                out.X = out.X(1:last_i,:);
                out.T = out.T(1:last_i,:);

            else
                out.X = [ext_out.X;out.X(1:last_i,:)];
                out.T = [ext_out.T;ext_out.T(end)+out.T(1:last_i,:)];

            end
        end
    end
end


