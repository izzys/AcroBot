function [ sim ] = Run( sim )
% Run the simulation until an event occurs
% Handle the event and keep running
    X = [];
    sim.Out.T = [];
    
    options=odeset('MaxStep',sim.tstep/10,'RelTol',.5e-7,'AbsTol',.5e-7,...
            'OutputFcn', @sim.Output_function, 'Events', @sim.Events);

    tspan = sim.tstart:sim.tstep:sim.tend;
    [TTemp,XTemp,TE,YE,IE] = ...
        ode45(@sim.Derivative,tspan,sim.IC,options); %#ok<ASGLU>
    
    if sim.infTime == 1
        TimeCond = true;
        sim.tend = sim.tend + TTemp(end)-tspan(1);
    else
        TimeCond = TTemp(end)<sim.tend;
    end
    
    % Save state and time
    X = [X; XTemp];
    sim.Out.T = [sim.Out.T; TTemp];

    while TimeCond && sim.StopSim == 0

        StepDone = 0;
        Xa = XTemp(end,:);
        for ev = 1:length(IE)
            
            % Is it a model event?
            ModEvID = find(IE(ev) == sim.ModEv,1,'first');
            if ~isempty(ModEvID)
                
                [sim.Mod,Xa(sim.ModCo)] = ...
                    sim.Mod.HandleEvent(ModEvID, XTemp(end,sim.ModCo),TTemp(end));
                
                % Call controller for model events:
                [sim.Con,Xa(sim.ConCo)] =  ...
                    sim.Con.HandleExtEvent(ModEvID, XTemp(end,:),TTemp(end));
                
                StepDone = 1; % model event is called when pendulum reaches -0.1 rad, this is the poincare section.
            end

            % Is it a controller event?
            ConEvID = find(IE(ev) == sim.ConEv,1,'first');
            if ~isempty(ConEvID)
                
                [sim.Con,Xa(sim.ConCo)] = ...
                    sim.Con.HandleEvent(ConEvID, XTemp(end,sim.ConCo),TTemp(end));

            end
            
        end
       
        sim.IC = Xa;
        
        if StepDone
            sim.ICstore(:,2:end) = sim.ICstore(:,1:end-1);
            sim.ICstore(:,1) = sim.IC';
            sim.StepsTaken = sim.StepsTaken+1;
            if ~sim.Graphics
            disp(['steps: ' num2str(sim.StepsTaken)])
            end
            sim = sim.CheckConvergence();
            sim.Out.PoincareSection(:,sim.StepsTaken) = sim.IC';
        end
        
        if sim.StopSim
            break;
        end

        % Continue simulation
        tspan = TTemp(end):sim.tstep:sim.tend;
        if length(tspan)<2
            % Can happen at the end of tspan
            break;
        end
        [TTemp,XTemp,TE,YE,IE] = ...
            ode45(@sim.Derivative,tspan,sim.IC,options); %#ok<ASGLU>
        
        if sim.infTime == 1
            TimeCond = true;
            sim.tend = sim.tend + TTemp(end)-tspan(1);
        else
            TimeCond = TTemp(end)<sim.tend;
        end
        
        % Save state and time
        X = [X; XTemp]; %#ok<AGROW>
        sim.Out.T = [sim.Out.T; TTemp];

    end
    
    % Prepare simulation output
    sim.Out.X = X;
    if ~isempty(sim.Period)
        sim.Out.Tend = sim.Out.T(end);
    else
        sim.Out.Tend = sim.tend;
    end

    sim.Out.nSteps = sim.StepsTaken;
    sim.Out.StepsSS = sim.stepsSS;

end

