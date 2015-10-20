function [ sim ] = Init( sim )
% Initialize simulation properties
    % Set states
    sim.stDim = sim.Mod.stDim + sim.Con.stDim; % state dimension
    sim.ModCo = 1:sim.Mod.stDim; % Model coord. indices
    sim.ConCo = sim.Mod.stDim+1:sim.stDim; % Contr. coord. indices

    % Set events
    sim.nEvents = sim.Mod.nEvents + sim.Con.nEvents;
    sim.ModEv = 1:sim.Mod.nEvents; % Model events indices
    sim.ConEv = sim.Mod.nEvents+1:sim.nEvents; % Contr. events indices
    
    %set sim IC:
    
    if strcmp(sim.Con.Controller_Type,'Hopf_adaptive') && sim.Con.NumOfNeurons>1
        sim.Con.IC = repmat(sim.Con.IC,sim.Con.NumOfNeurons,1);
    end
    
    sim.IC = [sim.Mod.IC ; sim.Con.IC];
    sim.StopSim = 0;
        
    % Set render params
    if sim.Graphics == 1
        if sim.Fig == 0
            sim.Once = 1;
        end
        
        % Init window size params
        scrsz = get(0, 'ScreenSize');
        if scrsz(3)>2*scrsz(4) % isunix()
            % If 2 screens are used in Linux
            scrsz(3) = scrsz(3)/2;
        end
        sim.FigWidth = scrsz(3)-500;
        sim.FigHeight = scrsz(4)-350;
        sim.AR = sim.FigWidth/sim.FigHeight;
        if isempty(sim.IC)
            [sim.COMx0,sim.COMy0] = sim.Mod.GetPos(zeros(1,sim.Mod.stDim),'COM');
        else
            [sim.COMx0,sim.COMy0] = sim.Mod.GetPos(sim.IC(sim.ModCo),'COM');
        end
        
        % Init world size params
        sim.FlMin = -0.2;
        sim.FlMax = 0.2;
        %sim.HeightMin = sim.COMy0-6/sim.AR*sim.Mod.l1;
        %sim.HeightMax = sim.COMy0+4/sim.AR*sim.Mod.l1;

    end
    
    
    % init model:
    sim.Mod.Torque = 0;
    sim.Mod.x0 = 0;
    sim.Mod.y0 = 0;   
    
    % init stats:
    sim.StepsTaken = 0;
    sim.ICstore = zeros(sim.stDim, sim.nICsStored);
    sim.stepsSS = zeros(1,sim.nICsStored-1);
    
    % Init sim.End result
    sim.Out.Torque = [];
    sim.Out.PoincareSection = [];
    sim.Out.Torque_time = [];
    sim.Out.Type = 0;
    sim.Out.Text = 'Reached end of tspan';
    
end

