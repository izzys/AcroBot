function [] = Render(sim,t,X,flag)
% Renders the simulation graphics
    switch flag
        case 'init'
            t = t(1);
            
            if sim.Once
                % Open new figure
                if sim.Fig
                    figure(sim.Fig);
                    if isempty(findobj(gcf,'Type','uicontrol'))
                        % Make window larger
                        set(sim.Fig,'Position', [100 200,...
                            sim.FigWidth sim.FigHeight]);
                    end
                else
                    sim.Fig = figure();
                    % Make window larger
                    set(sim.Fig,'Position', [100 100,...
                        sim.FigWidth sim.FigHeight]);
                end
                set(gca,'LooseInset',get(gca,'TightInset')*2)
                cla % clear previous render
                axis equal
                axis([-1 1 -1.5 0.5])
       
                % Initialize display timer
                sim.hTime = uicontrol('Style', 'text',...
                    'String', sprintf(sim.TimeStr,t),...
                    'HorizontalAlignment','left',...
                    'FontSize',11,...
                    'Units','normalized',...
                    'Position', [0.76 0.78 0.08 0.12],...
                    'backgroundcolor',get(gca,'color')); 
                
                % Initialize convergence display
                sim.hConv = uicontrol('Style', 'text',...
                    'String', sprintf(sim.ConvStr,1,'-'),...
                    'HorizontalAlignment','left',...
                    'FontSize',11,...
                    'Units','normalized',...
                    'Position', [0.76 0.7 0.08 0.12],...
                    'backgroundcolor',get(gca,'color')); 

                % Add a 'Stop simulation' button
                uicontrol('Style', 'pushbutton', 'String', 'Stop Simulation',...
                    'Units','normalized','FontSize',12,...
                    'Position', [0.75 0.92 0.2 0.06],...
                    'Callback', @sim.StopButtonCb);
                sim.Once = 0;

            end
    end
    
    if ishandle(sim.tCOM)==0
        sim.Once = 1;
        status = Render(sim,t,X,flag);
        return
    end
    
    if ~isempty(X)

        % Update model render
        sim.Mod = sim.Mod.Render(X(sim.ModCo));
        % Update environment render
        sim.Env = sim.Env.Render(sim.FlMin,sim.FlMax);
        % Update time display
        set(sim.hTime,'string',...
            sprintf(sim.TimeStr,t(1), int2str(sim.StepsTaken)) );
        % Update convergence display
        Period = find(sim.stepsSS>0,1,'first');
        if ~isempty(Period)
            diff = norm(sim.ICstore(:,1) - sim.ICstore(:,1+Period));
            set(sim.hConv,'string',...
                sprintf(sim.ConvStr,diff,int2str(Period)),...
                    'backgroundcolor',[0.5 1 0.5]);
        else
            diff = norm(sim.ICstore(:,1) - sim.ICstore(:,2));
            set(sim.hConv,'string',...
                sprintf(sim.ConvStr,diff,'-'),...
                    'backgroundcolor',get(gca,'color'));
        end

    end
    status = sim.StopSim;
    drawnow
end