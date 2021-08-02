
function X = catenary3D(pointA,pointB,length)

dist = norm(pointA-pointB);

if nargin < 3 %Determinar Comprimento automaticamente baseado na dist�ncia entre os pontos

    %Aplicando uma queda de 16% aproximadamente em rela��o ao comprimento
    %da linha. Coloquei 16% porque medi uma foto de linha de transmiss�o
    %com r�gua.
    queda = 0.16;
    length = 2*sqrt( (dist/2)^2 + (queda*dist)^2);
end


nPoints = ceil(10*norm(pointA-pointB)); % Determinar o n�mero de pontos do segmento
%Estou assumindo que teremos 100 pontos para cada 10 metros de dist�ncia
%linear

if pointA(1) ~= pointB(1)
    [X(1,:), X(3,:)] = catenary(pointA([1 3]), pointB([1 3]), length, nPoints);
    vecN = cross((pointA-pointB),[0 0 1])/norm(cross((pointA-pointB),[0 0 1]));
    angle = pi/2+atan2(vecN(2),vecN(1));
    
    % Rota��o em Z
    Rz = [cos(angle) -sin(angle) 0;
      sin(angle) cos(angle)  0;
      0         0          1];
    X(2,:) = 0*ones(1,nPoints);
    

    X = Rz*(X-[pointB(1) 0 pointB(3)]');
    X(1:2,:) = X(1:2,:)*(norm(pointA([1 2])-pointB([1 2]))/(pointA(1) - pointB(1)));
    X = X + pointB';
else
    [X(2,:), X(3,:)] = catenary(pointA([2 3]), pointB([2 3]), length, nPoints);
    vecN = cross((pointA-pointB),[0 0 1])/norm(cross((pointA-pointB),[0 0 1]));
    angle = atan2(vecN(2),vecN(1));
    % Rota��o em Z
    Rz = [cos(angle) -sin(angle) 0;
      sin(angle) cos(angle)  0;
      0         0          1];
    X(1,:) = 0*ones(1,nPoints);
 
    X = Rz*(X-[0 pointB(2) pointB(3)]');
    X(1:2,:) = X(1:2,:)*(norm(pointA([1 2])-pointB([1 2]))/(pointA(2) - pointB(2)));
    X = X + pointB';
end

end

% plot3(X(1,:),X(2,:),X(3,:))
% axis equal
% hold on
% grid on
% plot3(pointA(1),pointA(2),pointA(3),'*k')
% plot3(pointB(1),pointB(2),pointB(3),'*k')

















