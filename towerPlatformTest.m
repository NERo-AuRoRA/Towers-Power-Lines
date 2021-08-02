clc
clearvars
close all

for n = 1:3
    tower(n) = tPowerLine;
end

f1 = figure('Name','Simulação: Tarot 650-Sport & Gripper','NumberTitle','off');
f1.Position = [9 2 930 682];

figure(f1);

ax = gca;
ax.FontSize = 12;
xlabel({'$$x$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$y$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$z$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
view(3)
view(45,30)
grid on
hold on
grid minor

% Estilizando superficie
lighting phong;
material shiny;
lightangle(45,30)
light('Position',[-10 20 10]);

% axis([-2.5 2.5 -2 2 0 1])
set(gca,'Box','on');

set(gca, 'Color', 'none')

axis tight



tower(1).mCADplot;
tower(1).mCADcolor([0.7 0.8 0.8]);


tower(2).pPos.X(1:6) = [30 0 0 0 0 0]'; 
tower(2).mCADplot;
tower(2).mCADcolor([0.1 0.8 0.3]);


tower(3).pPos.X(1:6) = [60 10 5 0 0 pi/4]'; 
tower(3).mCADplot;
tower(3).mCADcolor([0.7 0.2 0.2]);



%%

lines = 1;

if ~strcmp(tower(1).pCAD.i3D{1}.Visible, 'on')
    for id = 1:length(tower(1).pCAD.i3D)
        tower(1).pCAD.i3D{id}.Visible = 'on';
        tower(2).pCAD.i3D{id}.Visible = 'on';
    end

    for id = 1:length(tower(1).pCAD.i3D)
        tower(1).pCAD.i3D{id}.Visible = 'on';
        tower(2).pCAD.i3D{id}.Visible = 'on';
        tower(3).pCAD.i3D{id}.Visible = 'on';
    end

    if lines
        for idx = 1:2
            line_01{idx} = catenary3D(tower(idx).pPos.XEa_F,tower(idx+1).pPos.XEa_T);
            line_02{idx} = catenary3D(tower(idx).pPos.XEb_F,tower(idx+1).pPos.XEb_T);
            line_03{idx} = catenary3D(tower(idx).pPos.XEc_F,tower(idx+1).pPos.XEc_T);
            line_04{idx} = catenary3D(tower(idx).pPos.XDa_F,tower(idx+1).pPos.XDa_T);
            line_05{idx} = catenary3D(tower(idx).pPos.XDb_F,tower(idx+1).pPos.XDb_T);
            line_06{idx} = catenary3D(tower(idx).pPos.XDc_F,tower(idx+1).pPos.XDc_T);

            hold on
            plot3(line_01{idx}(1,:),line_01{idx}(2,:),line_01{idx}(3,:),'-k','LineWidth',1);
            plot3(line_02{idx}(1,:),line_02{idx}(2,:),line_02{idx}(3,:),'-k','LineWidth',1);
            plot3(line_03{idx}(1,:),line_03{idx}(2,:),line_03{idx}(3,:),'-k','LineWidth',1);
            plot3(line_04{idx}(1,:),line_04{idx}(2,:),line_04{idx}(3,:),'-k','LineWidth',1);
            plot3(line_05{idx}(1,:),line_05{idx}(2,:),line_05{idx}(3,:),'-k','LineWidth',1);
            plot3(line_06{idx}(1,:),line_06{idx}(2,:),line_06{idx}(3,:),'-k','LineWidth',1);
            hold off
        end
    end



    axis tight
end

