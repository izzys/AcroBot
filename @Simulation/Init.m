function [ Sim ] = Init( Sim )
% Initialize simulation properties
    % Set states
    Sim.stDim = Sim.Mod.stDim + Sim.Con.stDim; % state dimension
    Sim.ModCo = 1:Sim.Mod.stDim; % Model coord. indices
    Sim.ConCo = Sim.Mod.stDim+1:Sim.stDim; % Contr. coord. indices

    % Set events
    Sim.nEvents = Sim.Mod.nEvents + Sim.Con.nEvents;
    Sim.ModEv = 1:Sim.Mod.nEvents; % Model events indices
    Sim.ConEv = Sim.Mod.nEvents+1:Sim.nEvents; % Contr. events indices
    
    %set Sim IC:
    
    if strcmp(Sim.Con.Controller_Type,'Hopf_adaptive') && Sim.Con.NumOfNeurons>1
        Sim.Con.IC = repmat(Sim.Con.IC,Sim.Con.NumOfNeurons,1);
    end
    
    Sim.IC = [Sim.Mod.IC ; Sim.Con.IC];
    Sim.StopSim = 0;
        
    % Set render params
    if Sim.Graphics == 1
        if Sim.Fig == 0
            Sim.Once = 1;
        end
        
        % Init window size params
        scrsz = get(0, 'ScreenSize');
        if scrsz(3)>2*scrsz(4) % isunix()
            % If 2 screens are used in Linux
            scrsz(3) = scrsz(3)/2;
        end
        Sim.FigWidth = scrsz(3)-500;
        Sim.FigHeight = scrsz(4)-350;
        Sim.AR = Sim.FigWidth/Sim.FigHeight;
        if isempty(Sim.IC)
            [Sim.COMx0,Sim.COMy0] = Sim.Mod.GetPos(zeros(1,Sim.Mod.stDim),'COM');
        else
            [Sim.COMx0,Sim.COMy0] = Sim.Mod.GetPos(Sim.IC(Sim.ModCo),'COM');
        end
        
        % Init world size params
        Sim.FlMin = -0.2;
        Sim.FlMax = 0.2;
        %Sim.HeightMin = Sim.COMy0-6/Sim.AR*Sim.Mod.l1;
        %Sim.HeightMax = Sim.COMy0+4/Sim.AR*Sim.Mod.l1;

    end
    
    
    % init model:
    Sim.Mod.Torque = 0;
    Sim.Mod.theta2_desired = Sim.Con.IC;
    Sim.Mod.x0 = 0;
    Sim.Mod.y0 = 0;   
    
    % init stats:
    Sim.StepsTaken = 0;
    Sim.ICstore = zeros(Sim.stDim, Sim.nICsStored);
    Sim.stepsSS = zeros(1,Sim.nICsStored-1);
    
    % init some controller params:
    Sim.Con.InitialPotentialNrg = Sim.Mod.GetNrg(Sim.IC,'potential');
    
    % Init Sim.End result
    Sim.Out.Torque = [];
    Sim.Out.Ep = [];
    Sim.Out.Ek = [];
    Sim.Out.Etot = [];
    Sim.Out.PoincareSection = [];
    Sim.Out.out_time = [];
    Sim.Out.Type = Sim.EndFlag_EndOfTime;
    Sim.Out.Text = 'Reached end of tspan';
    
end

