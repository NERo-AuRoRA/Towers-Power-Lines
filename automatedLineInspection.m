clearvars;close all


for n = 1:3
    tower(n) = tPowerLine;
end


f1 = figure('Name','Simulacao: Tarot 650-Sport & Gripper','NumberTitle','off');
f1.Position = [9 2 930 682];

A = ArDrone;
A.pSC.Kinematics_control = 0; 
A.pPos.X(1) = -10;
A.mCADplot


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
set(gca,'Box','on');
set(gca, 'Color', 'none')

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

%% ========================================================================
%% ========================================================================
% Definição da cena:
%Diametro do A real (63 cm)
dD = .7;
p0 = [-10,0,1.5]; n0 = [1,0,0];% ponto inicial

% Bambolês:
b1 = tower(1).pPos.X([1 2 3])' + [0 10 12.5];
n1 = [1,0,0];

b2 = tower(2).pPos.X([1 2 3])' + [0 10 12.5];
n2 = [1,0,0];

b3 = tower(3).pPos.X([1 2 3])' + [0 10 12.5];
n3 = [1,0,0];

%% ida
b4 = b3 + [10 -10 0];
b5 = b3 - [0 b3(2) 0];
b6 = b2 -[0 2*b2(2) 0];
b7 = b1 -[0 2*b1(2) 0];

n1 =(b2-p0)./norm(b2-p0);
n2 =(b3-b2)./norm(b3-b2);
n3 =(b4-b3)./norm(b4-b3);

u1 = b1 +[1 0 0];
u1 = n1.*(u1);
u1 = u1./norm(u1);
% [sum(n1.*b1)/n1(1) 0 0];
v1 = b1 +[0 1 0];
v1 = n1.*(v1);
v1 = v1./norm(v1);
% [0 sum(n1.*b1)/n1(2) 0];

% v1 = [1,0,1];    
% u1 = [0,1,0.5];

% v2 = [0.5,0,0.5];   
% u2 = [0,0.5,0.5];
v2 = [0.5,0,0.5];   
u2 = [0,0.5,0.5];


v3 = [0,-1,0];      
u3 = [0,0,1];




% Conjunto dos bambolês:
Bmb = [b1;b2;b3;b4;b5;b6;b7]; 
n1 = [1 0 0];
Nb = -[n1;n2;n3;-[0 1 0];-n1;-n2;-n3];

% linhas = [{line_01{1}},{line_02{1}},{line_03{1}},{line_01{2}},{line_02{2}},{line_03{2}}];
%     {line_04{1}},{line_05{1}},{line_06{1}}];

linhas = [{line_04{1}},{line_04{2}},{flip(line_01{2},2)},{flip(line_01{1},2)}];

bTorre = [];
for ii= 1:size(linhas,2)
    % Pontos a usar: 1, round(mean(size(linhas(ii){:,ii},1))
    Q1 = round(size(linhas{ii},2)/4);
    Q2 = round(size(linhas{ii},2)/2);
    Q3 = round(size(linhas{ii},2)*3/4);
    
%     linhas{ii}(:,MEIO)
    vec = [linhas{ii}(:,1) linhas{ii}(:,Q1) linhas{ii}(:,Q2) linhas{ii}(:,Q3) linhas{ii}(:,end)];
    
    if ii==1 || ii==2
        vec = vec + [0 5 -1]';
    else 
        vec = vec + [0 -5 -1]';
    end   
     
    bTorre = [bTorre vec];
end


nTorre = [1 0 0]';
for jj = 2:size(bTorre,2)
    nTorre = [nTorre, -bTorre(:,jj)-bTorre(:,jj-1)./norm(bTorre(:,jj)-bTorre(:,jj-1))];
end

% bTorre = bTorre';
% MAX = size(Bmb,1);
% C1= 1;
% CC = 1;
% while CC <= 15
%     Bmb = [Bmb(1:C1,:); bTorre(CC,:); bTorre(CC+1,:); bTorre(CC+2,:); Bmb(C1:end,:)];
%        
%     C1= C1+4;
%     CC = CC+3;
% end
% 
% nTorre = nTorre';
% MAX = size(Nb,1);
% C1= 1;
% CC = 1;
% while CC <= 15
%     Nb = [Nb(1:C1,:); nTorre(CC,:); nTorre(CC+1,:); nTorre(CC+2,:); Nb(C1:end,:)];
%        
%     C1= C1+4;
%     CC = CC+3;
% end
Bmb = bTorre';
Nb = nTorre';
Bmb = [Bmb(1:10,:); [70 10 18]; Bmb(11:end,:)];
Nb = [Nb(1:10,:); [0 -1 0]; Nb(11:end,:)];

Vb = [];
Ub = [];
for SEILA = 1:size(Bmb,1)
   Vb = [Vb;v1]; 
   Ub = [Ub;u1];
end
Vb(2,:) = v2;
Ub(2,:) = u2;
% Vb = [v1;v2;v1;v1;v1;v1;v1];  
% Ub = [u1;u2;u1;u1;u1;u1;u1];

Curva =[];        Curva.Vmax = .15;
%% Plots ==================================================================
[CB,dCB,Curva] = CurvaBmbLines(p0,n0,Bmb,-Nb,Vb,Ub,Curva,0.05);


% for bi = 1:size(Nb,1)
%     plot3(Bmb(bi,1),Bmb(bi,2),0,'b+','MarkerSize',10,'LineWidth',2);
%     plot3([Bmb(bi,1),Bmb(bi,1)],[Bmb(bi,2),Bmb(bi,2)],[Bmb(bi,3),0],'b--')
% end
% plot3(b2(1),b2(2),0,'b+','MarkerSize',10,'LineWidth',2);
% plot3([b2(1),b2(1)],[b2(2),b2(2)],[b2(3),0],'b--')
% 
% plot3(b3(1),b3(2),0,'b+','MarkerSize',10,'LineWidth',2);
% plot3([b3(1),b3(1)],[b3(2),b3(2)],[b3(3),0],'b--')

% plot3([0,0],[0,0],[0.75,0],'k--')

% %
% xlim([-10 5]); ylim([-10 5]); zlim([-1 5]);
xlim([0 5]); ylim([-10 0]); zlim([-1 5]);
%%
t = tic;
tc = tic;
tp = tic;
rho = 0;
Curva.Vmax = 0.3;
XX = [];
while toc(t)< 5*60
%     Curva.Pos < Curva.Size
    if toc(tc) > 1/30
        % Atualiza tempo:
        tc = tic;  tt = toc(t); 
        
        % Mostrar progresso:
        clc 
        fprintf('Posicao no caminho: %i \n',Curva.Pos)
        fprintf('Percorrendo: %0.2g%% \n',(Curva.Pos/Curva.Size)*100)
        
        A.rGetSensorData % Get data 
        
        % Controlador
%         [A,Curva,rho] = cPathFollowingLines(A,Curva,.5);
%         [A,Curva,rho] = cPathFollowingLines(A,Curva);
        A.pPos.Xd(6) = atan2(Curva.dX(2,Curva.Pos),Curva.dX(1,Curva.Pos));
        A.pPos.Xr([1:3,6:9]) = [A.pPos.Xd(1:3);A.pPos.Xd(6);A.pPos.Xd(7:9)];
        
%         if rho<0.35
%             A.pSC.Kinematics_control = 1;
%         else
%             A.pSC.Kinematics_control = 0; 
%         end
             
%         A.pSC.tcontrole = tc;
        A = cUnderActuatedController(A); 
        A.rSendControlSignals

        % Histórico:  12        12      1   
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        Curva.rho = [Curva.rho,rho];
        
        try 
            delete(pDrone)
        end


        A.mCADplot; 
        pDrone = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'ok','MarkerSize',10);
        drawnow
    end
                 
end
