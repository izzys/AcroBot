classdef Simulation < handle & matlab.mixin.Copyable
    
   properties(Constant)
 
        % End flags: 
        EndFlag_EndOfTime = 0;
        EndFlag_GoalHeightReached = 0;
 
    end
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
        % Set EndCond to run the Sim until:
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
        function Sim = Simulation(varargin)

                    Sim.Mod = Model();
                    Sim.Con = Controller();
                    Sim.Env = Terrain();
           
        end
        
        % Make a deep copy of a handle object.
        function SimDC = deepcopy(Sim)
            % Instantiate new object of the same class.
            SimDC = copy(Sim);
            SimDC.Mod = copy(Sim.Mod);
            SimDC.Con = copy(Sim.Con);
            SimDC.Env = copy(Sim.Env);
        end
        
        function Sim = SetEndCond(Sim, value)
            L = length(value);
            if L<1
                error('Invalid input for EndCond');
            end
            
            if value(1) == 1
                Error = ['When setting EndCond to 1,',...
                           'a value for num. steps is also needed',...
                           '\nPlease use Sim.EndCond = [1,nsteps]'];
                if L<2
                    error(Error);
                else
                    if ~isnumeric(value(2)) || value(2)<1
                        error(Error);
                    end
                end
            end
            
            Sim.EndCond = value;
        end
        
        function Sim = SetTime(Sim,tstart,tstep,tend)
            if nargin~=4
                error(['Set time expects 3 input arguments',...
                    ' but was provided with ',num2str(nargin)]);
            end
            Sim.tstart = tstart;
            Sim.tstep_normal = tstep;
         %   Sim.tstep_small = tstep/3;
            Sim.tstep = tstep;
            if isnumeric(tend)
                if tend<=tstart+tstep
                    error('tend is too close to tstart');
                else
                    Sim.tend = tend;
                end
                Sim.infTime = 0;
            else
                if strcmp(tend,'inf')
                    % Simulation will run for indefinite time
                    Sim.infTime = 1;
                    Sim.tend = 10;
                end
            end
            Sim.Out.Tend = Sim.tend;
        end
        
        function [Xt] = Derivative(Sim,t,X)
            
            Xt = [Sim.Mod.Derivative(t,X(Sim.ModCo));
                  Sim.Con.Derivative(t,X(Sim.ConCo))];
        end

        function [value, isterminal, direction] = Events(Sim, t, X) 
            
            value = zeros(Sim.nEvents,1);
            isterminal = ones(Sim.nEvents,1);
            direction = zeros(Sim.nEvents,1);

            % Call model event function
            [value(Sim.ModEv), isterminal(Sim.ModEv), direction(Sim.ModEv)] = ...
                Sim.Mod.Events(t,X(Sim.ModCo), Sim.Env);
            
            % Call controller event function
            [value(Sim.ConEv), isterminal(Sim.ConEv), direction(Sim.ConEv)] = ...
                Sim.Con.Events(t,X(Sim.ConCo));
            

        end
        
        function [status] = Output_function(Sim,t,X,flag) 
           
           switch flag
               
               case 'init'
                  
                  % get controler action - first time:
                  Sim.Mod.Torque = Sim.Con.Get_Torque(Sim.IC,Sim.Mod,t(1)); 
                  
                  % save for out:
                  Sim.Out.Torque = [Sim.Out.Torque ; Sim.Mod.Torque];
                  Sim.Out.out_time  = [Sim.Out.out_time  t(1)];
                                    
                  Sim.Out.Ep = [Sim.Out.Ep ;   Sim.Mod.GetNrg(Sim.IC,'potential')];
                  Sim.Out.Ek = [Sim.Out.Ek ; Sim.Mod.GetNrg(Sim.IC,'kinetic') ];
                  Sim.Out.Etot = [Sim.Out.Etot ; Sim.Mod.GetNrg(Sim.IC,'total') ];
        
               case 'done'

                   return;
                            
               otherwise
                   
                  % get controler action:
                  Sim.Mod.Torque = Sim.Con.Get_Torque(X(:,end),Sim.Mod,t(end));
                  
                  % save for out:
                  Sim.Out.Torque = [Sim.Out.Torque ; Sim.Mod.Torque];
                  Sim.Out.out_time  = [Sim.Out.out_time  t(end)];
                  
                  Sim.Out.Ep = [Sim.Out.Ep ;   Sim.Mod.GetNrg(X(:,end),'potential')];
                  Sim.Out.Ek = [Sim.Out.Ek ; Sim.Mod.GetNrg(X(:,end),'kinetic') ];
                  Sim.Out.Etot = [Sim.Out.Etot ; Sim.Mod.GetNrg(X(:,end),'total') ];
                  
           end
           
           if Sim.Graphics == 1
             Sim.Render(t,X,flag);
           end
           
           status = Sim.StopSim; 
            
        end

        function StopButtonCb(Sim, hObject, eventdata, handles) %#ok<INUSD>
            if Sim.StopSim == 0
                Sim.StopSim = 1;
                Sim.Out.Type = -1;
                Sim.Out.Text = 'Simulation stopped by user';
                set(hObject,'String','Close Window');
            else
                close(Sim.Fig)
            end
        end  
        
      
        
        function out = JoinOuts(Sim,ext_out,last_i)
            if nargin<3
                last_i = length(Sim.Out.T);
            end
            
            out = Sim.Out;
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


