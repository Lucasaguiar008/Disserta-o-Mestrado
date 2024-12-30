
function [] = DynamicWindowApproachSample(rota_vant,Obstaculos,contador,titulo,numfig)
    
    disp('Abordagem da Janela Dinâmica iniciado!!')

    % Definindo a posição de inicio 
    pos1(1) = rota_vant(contador,1);
    pos1(2) = rota_vant(contador,2);

    % Definindo a posição do alvo
    pos2(1) = rota_vant(contador+1,1);
    pos2(2) = rota_vant(contador+1,2);       
    
    % Estado inicial do robô [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    x = [pos1(1) pos1(2) pi/2 1 100]';
    
    % Posição do destino [x(m),y(m)]
    goal = [pos2(1),pos2(2)];

    % Corrigindo o angulo
    % Definir os pontos A e B
    A = [x(1) x(2)];
    B = goal;
    
    % Calcular o ângulo entre os pontos A e B em radianos
    novo_angulo = atan2(B(2) - A(2), B(1) - A(1));
    x(3) = novo_angulo;
    
    % Posição dos obstáculos [x(m) y(m)]
    obstacle = Obstaculos;  
    
    % Pisção dos obstáculos dinâmicos
    global z;
    z = 30;
    a=0; b=9;
    obst_Din = a + (b-a).*rand(z,2);
    
    % Raio de obstáculo para detecção de colisão
    obstacleR = 0.1;
    
    % tempo de passo [s]
    global dt; dt = 0.1;
    
    % Modelo de mecânica do robô     
    Kinematic=[1.0,...             % Velocidade máxima [m/s]
               toRadian(20.0),... % Velocidade máxima de giro [rad/s]
               0.2,...             % Aceleração e desaceleração máximas [m/s²]
               toRadian(50.0),...  % Velocidade máxima de giro de aceleração e desaceleração [rad/s²]
               0.01,...            % Resolução de velocidade [m/s]
               toRadian(1)];      % Resolução de velocidade de retorno [rad/s]]

    % Parâmetros da função de avaliação
    evalParam=[0.3,... %heading
               0.8,... %dist 0.2
               0.1,... %velocity
               2.0]; %predictDT
    
    % Tamanho da área de simulação [xmin xmax ymin ymax]
    area = [-1 16 -1 11];
    
    % resultado da simulação
    result.x = [];    
    
    % Movimento principal
    for i = 1:900000
        
        % DWA - Calculando valores de entrada
        [u, traj] = DynamicWindowApproach(x, Kinematic, goal, evalParam, obstacle, obstacleR);
        
        % Locomoção por modelo de movimento
        x = f(x, u);
        
        % Salvando resultados de simulação
        result.x = [result.x; x'];
        
        % Julgamento de meta
        if norm(x(1:2) - goal') < 0.2
            disp('Chegou ao objetivo!!');

            % Verifica se já visitou todas as rotas
            if length(rota_vant)>2
                rota_vant(1,:)=[]; %apagando a primeira linha                

                % Definindo a posição de inicio 
                pos1(1) = rota_vant(contador,1);
                pos1(2) = rota_vant(contador,2);
            
                % Definindo a posição do alvo
                pos2(1) = rota_vant(contador+1,1);
                pos2(2) = rota_vant(contador+1,2);
                                
                % Posição do destino [x(m),y(m)]                
                goal = [pos2(1),pos2(2)];                

                % Definir os pontos A e B
                A = [x(1) x(2)];
                B = goal;
                
                % Calcular o ângulo entre os pontos A e B em radianos
                novo_angulo = atan2(B(2) - A(2), B(1) - A(1));
                x(3) = novo_angulo;
               
            else
                disp('Fim da rota do VANT');
                break;                
            end

        end
        
        % ==== Animação ====
        hold off;

        figure(numfig)        

        % Plotando o mapa                       
        % Galpão inicial e final    
        galpaoIncicial = [1,1];
        plot(galpaoIncicial(1),galpaoIncicial(2),'b--o','MarkerSize' ,10);
        text(galpaoIncicial(1)+0.2,galpaoIncicial(2)-0.2,'Galpao Inicial')
        hold on
        
        galpaoFinal = [13,9];
        plot(galpaoFinal(1),galpaoFinal(2),'r--o','MarkerSize' ,10);
        text(galpaoFinal(1)+0.2,galpaoFinal(2)+0.2,'Galpao Final')
        
        % waypoints
        WayPoints = [6,1; 8,9; 4,8; 13,6; 2,9; 1,4; 3,4; 11,7;5,3;4,6;10,2;7,5;8,4;10,6];
        plot(WayPoints(:,1),WayPoints(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor','y');
        x_aux = WayPoints(:,1);
        y_aux = WayPoints(:,2);
        z_aux = (2:1:size(WayPoints,1)+1)';
        text(x_aux+0.2,y_aux,"WP:"+ num2str(z_aux))
        
        %Obstaculos    
        plot(Obstaculos(:,1),Obstaculos(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0.5, 0.5, 0.5],'MarkerFaceColor',[0.5, 0.5, 0.5]);
        
        xlim([(min(WayPoints(:,1))-1.5) (max(WayPoints(:,1))+1.5)])
        ylim([(min(WayPoints(:,2))-1.5) (max(WayPoints(:,2))+1.5)])

        % Objetivo
        plot(goal(1), goal(2), '*g');
                            
        title(titulo)
    
    %%
        
        % comprimento da seta
        ArrowLength = 0.5;
        
        % robô
        quiver(x(1), x(2), ArrowLength*cos(x(3)), ArrowLength*sin(x(3)), 'or');
        hold on;
        plot(result.x(:,1), result.x(:,2), '-b');
        hold on;
        
        % Explore a representação da trajetória
        if ~isempty(traj)
            for it = 1:length(traj(:,1))/5
                ind = 1 + (it-1)*5;
                plot(traj(ind,:), traj(ind+1,:), '-g');
                hold on;
            end
        end
        axis(area);
        %grid on;
        drawnow;
        
        
        % Obstáculos dinâmicos 
        plot(obst_Din(:,1), obst_Din(:,2), '*k');
        hold on;

        a=-1; b=1;
        ax = a + (b-a).*rand(z,1);
        ay = a + (b-a).*rand(z,1);   
        eixo = 1;
        for cont=1:z
            acelerador = 0.01;
            if obst_Din(cont,eixo)>= randi(5)
                obst_Din(cont,eixo)=obst_Din(cont,eixo)-acelerador;
            else
                obst_Din(cont,eixo)=obst_Din(cont,eixo)+acelerador;
            end
        end
        
        % Atualizando os obstáculos
        obstacle = [Obstaculos;obst_Din] ;
        
    end
   
end

%% ===============================================================
%                       FUNÇÕES AUXILIARES
% ===============================================================

function [u, trajDB] = DynamicWindowApproach(x, model, goal, evalParam, ob, R)
    %DWA - Calcula os valores de entrada
    
    % Dynamic Window [vmin,vmax,ωmin,ωmax]
    Vr = CalcDynamicWindow(x, model);
    % Calcula a função de avaliação
    [evalDB, trajDB] = Evaluation(x, Vr, goal, ob, R, model, evalParam);
    
    if isempty(evalDB)
        disp('sem caminho para o objetivo!!');
        u = [0; 0];
        return;
    end
    
    % Normaliza cada função de avaliação
    evalDB = NormalizeEval(evalDB);
    
    % Calcula o valor final de avaliação
    feval = [];
    for id = 1:length(evalDB(:,1))
        feval = [feval; evalParam(1:3) * evalDB(id,3:5)'];
    end
    evalDB = [evalDB feval];
    
    % Calcula o valor máximo da avaliação
    [maxv, ind] = max(feval);
    % Retorna os valores de entrada correspondentes ao maior valor de avaliação
    u = evalDB(ind,1:2)';

end

function [evalDB, trajDB] = Evaluation(x, Vr, goal, ob, R, model, evalParam)
    % Calcula a função de avaliação para cada trajetória
    evalDB = [];
    trajDB = [];
    
    for vt = Vr(1):model(5):Vr(2)
        for ot = Vr(3):model(6):Vr(4)
            % Estima a trajetória
            [xt, traj] = GenerateTrajectory(x, vt, ot, evalParam(4), model);
            % Calcula cada função de avaliação
            heading = CalcHeadingEval(xt, goal);
            dist = CalcDistEval(xt, ob, R);
            vel = abs(vt);
            
            evalDB = [evalDB; [vt ot heading dist vel]];
            trajDB = [trajDB; traj];     
        end
    end
end

function EvalDB = NormalizeEval(EvalDB)
    % Normaliza os valores da função de avaliação
    if sum(EvalDB(:,3)) ~= 0
        EvalDB(:,3) = EvalDB(:,3) / sum(EvalDB(:,3));
    end
    if sum(EvalDB(:,4)) ~= 0
        EvalDB(:,4) = EvalDB(:,4) / sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5)) ~= 0
        EvalDB(:,5) = EvalDB(:,5) / sum(EvalDB(:,5));
    end
end

function [x, traj] = GenerateTrajectory(x, vt, ot, evaldt, model)
    % Gera dados de trajetória
    global dt;
    time = 0;
    u = [vt; ot]; % Valores de entrada
    traj = x; % Dados de trajetória
    while time <= evaldt
        time = time + dt; % Atualiza o tempo de simulação
        x = f(x, u); % Transição de acordo com o modelo de movimento
        traj = [traj x];
    end
end

function stopDist = CalcBreakingDist(vel, model)
    % Calcula a distância de frenagem de acordo com o modelo de dinâmica
    global dt;
    stopDist = 0;
    while vel > 0
        stopDist = stopDist + vel * dt; % Calcula a distância de frenagem
        vel = vel - model(3) * dt; % Princípio da velocidade
    end
end

function dist = CalcDistEval(x, ob, R)
    % Calcula a avaliação de distância em relação aos obstáculos
    dist = 2;
    for io = 1:length(ob(:,1))
        disttmp = norm(ob(io,:) - x(1:2)') - R;
        if dist > disttmp
            dist = disttmp;
        end
    end
end

function heading = CalcHeadingEval(x, goal)
    % Calcula a função de avaliação de heading
    theta = toDegree(x(3)); % Orientação do robô
    goalTheta = toDegree(atan2(goal(2) - x(2), goal(1) - x(1))); % Orientação do objetivo
    
    if goalTheta > theta
        targetTheta = goalTheta - theta;
    else
        targetTheta = theta - goalTheta;
    end
    
    heading = 180 - targetTheta;
end

function Vr = CalcDynamicWindow(x, model)
    % Calcula a Dynamic Window a partir do modelo e do estado atual
    global dt;
    % Janela Dinâmica do modelo do veículo
    Vs = [0 model(1) -model(2) model(2)];
    
    % Janela Dinâmica do modelo de movimento
    Vd = [x(4) - model(3)*dt x(4) + model(3)*dt x(5) - model(4)*dt x(5) + model(4)*dt];
    
    % Calcula a Dynamic Window final
    Vtmp = [Vs; Vd];
    Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
    % [vmin,vmax,ωmin,ωmax]
end

function x = f(x, u)
    % Modelo de Movimento
    global dt;
     
    F = [1 0 0 0 0
         0 1 0 0 0
         0 0 1 0 0
         0 0 0 0 0
         0 0 0 0 0];
     
    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];
    
    x = F*x + B*u;
    end

function radian = toRadian(degree)
    % grau para radiano
    radian = degree/180*pi;
end

function degree = toDegree(radian)
    % radiano para grau
    degree = radian/pi*180;
end
