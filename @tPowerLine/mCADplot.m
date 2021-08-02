function mCADplot(uav)
% Plot Tarot 650-Sport CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T


uav.pPos.Xc = uav.pPos.X;


if uav.pCAD.flagCreated == 0
    uav.pPos.psiHel = 0;
    
    mCADmake(uav)
    mCADplot(uav)
    
    drawnow limitrate nocallbacks
    
else
        
    %------------------------------------------------------------
    
    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot uav.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[uav.pCAD.obj{1}.v; ones(1,size(uav.pCAD.obj{1}.v,2))];
    uav.pCAD.i3D{1}.Vertices = vertices(1:3,:)';
    
    
    vertBase = H*[uav.pCAD.i3D{length(uav.pCAD.obj)+1}.Vertices'; ones(1,size(uav.pCAD.i3D{length(uav.pCAD.obj)+1}.Vertices,1))];
    uav.pCAD.i3D{length(uav.pCAD.obj)+1}.Vertices = vertBase(1:3,:)';
    
    
    % --- Dados da Linha:

    uav.pPos.verts = H*[uav.pPos.verts'; ones(1,size(uav.pPos.verts,1))];

    % Pontas da Frente:
    uav.pPos.XEa_F = uav.pPos.verts(1:3,1)';

    uav.pPos.XEb_F = uav.pPos.verts(1:3,2)';

    uav.pPos.XEc_F = uav.pPos.verts(1:3,3)';

    uav.pPos.XDa_F = uav.pPos.verts(1:3,4)';

    uav.pPos.XDb_F = uav.pPos.verts(1:3,5)';

    uav.pPos.XDc_F = uav.pPos.verts(1:3,6)';


    % Pontas de Trás:
    uav.pPos.XEa_T = uav.pPos.verts(1:3,7)';

    uav.pPos.XEb_T = uav.pPos.verts(1:3,8)';

    uav.pPos.XEc_T = uav.pPos.verts(1:3,9)';

    uav.pPos.XDa_T = uav.pPos.verts(1:3,10)';

    uav.pPos.XDb_T = uav.pPos.verts(1:3,11)';

    uav.pPos.XDc_T = uav.pPos.verts(1:3,12)';
    
    
    

end


end

% =========================================================================
function mCADmake(uav)

for i = 1:length(uav.pCAD.obj)
    
    hold on
    uav.pCAD.i3D{i} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{i}.FaceColor = 'flat';
    uav.pCAD.i3D{i}.EdgeColor = 'none';
    uav.pCAD.i3D{i}.FaceAlpha = 1.0;
    uav.pCAD.i3D{i}.Visible = 'off';
    % light;

end
    

% - Desenhando uma plataforma em baixo da torre:
hold on
uav.pCAD.i3D{length(uav.pCAD.obj)+1} = patch('Vertices',4*[-1 -1 0; -1 1 0; 1 1 0; 1 -1 0],'Faces',[1 2 3 4 1],'Marker','o','MarkerSize',7.5);
hold off

% uav.pCAD.i3D{length(uav.pCAD.obj)+1}.FaceColor = 'flat';
uav.pCAD.i3D{length(uav.pCAD.obj)+1}.EdgeColor = 'none';
uav.pCAD.i3D{length(uav.pCAD.obj)+1}.FaceAlpha = 0.5;
uav.pCAD.i3D{length(uav.pCAD.obj)+1}.Visible = 'off';
uav.pCAD.i3D{length(uav.pCAD.obj)+1}.MarkerFaceColor = 'blue';
uav.pCAD.i3D{length(uav.pCAD.obj)+1}.MarkerEdgeColor = 'k';


uav.pCAD.flagCreated = 1;



end