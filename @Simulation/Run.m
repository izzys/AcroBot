function [ Sim ] = Run( Sim )
% Run the simulation until an event occurs
% Handle the event and keep running
    X = [];
    Sim.Out.T = [];
    
    options=odeset('MaxStep',Sim.tstep/10,'RelTol',.5e-7,'AbsTol',.5e-7,...
            'OutputFcn', @Sim.Output_function, 'Events', @Sim.Events);

    tspan = Sim.tstart:Sim.tstep:Sim.tend;
    [TTemp,XTemp,TE,YE,IE] = ...
        ode45(@Sim.Derivative,tspan,Sim.IC,options); %#ok<ASGLU>
    
    if Sim.infTime == 1
        TimeCond = true;
        Sim.tend = Sim.tend + TTemp(end)-tspan(1);
    else
        TimeCond = TTemp(end)<Sim.tend;
    end
    
    % Save state and time
    X = [X; XTemp];
    Sim.Out.T = [Sim.Out.T; TTemp];

    while TimeCond && Sim.StopSim == 0

        Xa = XTemp(end,:);
        for ev = 1:length(IE)
            
            % Is it a model event?
            ModEvID = find(IE(ev) == Sim.ModEv,1,'first');
            if ~isempty(ModEvID)
                
                [Sim.Mod,Xa(Sim.ModCo)] = ...
                    Sim.Mod.HandleEvent(ModEvID, XTemp(end,Sim.ModCo),TTemp(end));
                
                % Call controller for model events:
                [Sim.Con,Xa(Sim.ConCo)] =  ...
                    Sim.Con.HandleExtEvent(ModEvID, XTemp(end,:),TTemp(end));
                
                if ModEvID==1
                    Sim.StopSim = 1;
                    Sim.Out.Type = Sim.EndFlag_GoalHeightReached;
                    Sim.Out.Text = 'Reached goal height';
                end
                
                if ModEvID==2  %dq1 = 0: (from positive)
                  %  disp(['dq1=0 from pos. : Torque on.  TimeStamp: ' num2str(TTemp(end))])
                    Sim.Mod.theta2_desired = Xa(Sim.ConCo);
                end 
                
                if ModEvID==3  %dq1 = 0: (from negative)
                 %   disp(['dq1=0 from neg. : Torque on.  TimeStamp: ' num2str(TTemp(end))])
                    Sim.Mod.theta2_desired = Xa(Sim.ConCo);
                end 
                
                if ModEvID==4 %q2-q2d = 0:
               %     disp(['q2 = q2d: Torque off.  TimeStamp: ' num2str(TTemp(end))])
                end
 
            end

            % Is it a controller event?
            ConEvID = find(IE(ev) == Sim.ConEv,1,'first');
            if ~isempty(ConEvID)
                
                [Sim.Con,Xa(Sim.ConCo)] = ...
                    Sim.Con.HandleEvent(ConEvID, XTemp(end,Sim.ConCo),TTemp(end));

            end
            
        end
       
        Sim.IC = Xa;
        
        if Sim.StopSim
            break;
        end

        % Continue simulation
        tspan = TTemp(end):Sim.tstep:Sim.tend;
        if length(tspan)<2
            % Can happen at the end of tspan
            break;
        end
        [TTemp,XTemp,TE,YE,IE] = ...
            ode45(@Sim.Derivative,tspan,Sim.IC,options); %#ok<ASGLU>
        
        if Sim.infTime == 1
            TimeCond = true;
            Sim.tend = Sim.tend + TTemp(end)-tspan(1);
        else
            TimeCond = TTemp(end)<Sim.tend;
        end
        
        % Save state and time
        X = [X; XTemp]; %#ok<AGROW>
        Sim.Out.T = [Sim.Out.T; TTemp];

    end
    
    % Prepare simulation output
    Sim.Out.X = X;
    if ~isempty(Sim.Period)
        Sim.Out.Tend = Sim.Out.T(end);
    else
        Sim.Out.Tend = Sim.tend;
    end

    Sim.Out.nSteps = Sim.StepsTaken;
    Sim.Out.StepsSS = Sim.stepsSS;

end

