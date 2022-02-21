function [Drone,Path] = cPathFollowingOFICIAL(Drone,Path)


% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0; 0 0.2  0; 0 0 0.20];
Khr2 = [0.75 0 0; 0 0.75 0; 0 0 0.75];
% Xd = zeros(12,1);

% Navegar para o primeiro ponto do caminho
% Verificar se o erro entre o caminho e a posição atual do robô é inferior
% a um limiar
if norm(Drone.pPos.X(1:3)-Path.X(1:3,Path.Pos)) < 0.05
    Path.Pos = Path.Pos + 1;    
    if Path.Pos > Path.Size
        Path.Pos = 1;
    end
end

% Cálculo do ponto mais próximo ao Rota em coordenadas polares
rho   = norm(Path.X(1:3,Path.Pos)-Drone.pPos.X(1:3));
alpha = atan2(Path.dX(3,Path.Pos),norm(Path.dX(1:2,Path.Pos)));
beta  = atan2(Path.dX(2,Path.Pos),Path.dX(1,Path.Pos));

% Cálculo da velocidade do robô sobre o Rota
% Velocidade Desejada muda de acordo com o erro de distância
V = Path.Vmax/(1+2*rho);

Path.dXr = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha)];

Xtil = Path.X(1:3,Path.Pos) - Drone.pPos.X(1:3)

% Modelo Cinemático
KM = [cos(Drone.pPos.X(6)) -sin(Drone.pPos.X(6)) 0; sin(Drone.pPos.X(6)) cos(Drone.pPos.X(6)) 0; 0 0 1];
uSC = KM\(Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3])));

Drone.pPos.Xd(1:3) = Path.X(1:3,Path.Pos);
Drone.pPos.Xd(7:9) = Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3]));
