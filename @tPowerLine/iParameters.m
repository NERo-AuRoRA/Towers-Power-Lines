function iParameters(tarot)

tarot.pPar.Model = 'tarot'; % robot model

% Sample time
tarot.pPar.Ts = 0.1; % For numerical integration
tarot.pPar.ti = tic; % Flag time

% Dynamic Model Parameters 
tarot.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration


% Pontas da Frente:
tarot.pPos.XEa_F = [0.850 -3.610 13.300];

tarot.pPos.XEb_F = [1.250 -2.725  9.200];

tarot.pPos.XEc_F = [1.250 -5.400  9.200];

tarot.pPos.XDa_F = [0.850  3.610 13.300];

tarot.pPos.XDb_F = [1.250  2.725  9.200];

tarot.pPos.XDc_F = [1.250  5.400  9.200];


% Pontas de Trás:
tarot.pPos.XEa_T = [-0.920 -3.610 13.300];

tarot.pPos.XEb_T = [-1.340 -2.725  9.200];

tarot.pPos.XEc_T = [-1.340 -5.400  9.200];

tarot.pPos.XDa_T = [-0.920  3.610 13.300];

tarot.pPos.XDb_T = [-1.340  2.725  9.200];

tarot.pPos.XDc_T = [-1.340  5.400  9.200];

%% - Matriz de pontos:
tarot.pPos.verts = [tarot.pPos.XEa_F;
                    tarot.pPos.XEb_F;
                    tarot.pPos.XEc_F;
                    tarot.pPos.XDa_F;
                    tarot.pPos.XDb_F;
                    tarot.pPos.XDc_F;
                    tarot.pPos.XEa_T;  % -----
                    tarot.pPos.XEb_T;
                    tarot.pPos.XEc_T;
                    tarot.pPos.XDa_T;
                    tarot.pPos.XDb_T;
                    tarot.pPos.XDc_T];

