
function [result] = mutipleDWA_CBA_corrigido(Nv,rotas,Obstaculos,titulo,numfig,gravar,nomeVideo,wp_visitados)
    
    disp('Abordagem da Janela Dinâmica iniciado!!')    
    cor = ['r', 'b' , 'g','k','c','y','m','g']; % Cores dos plots dos vants   
    for i=1:Nv  
        start{i} = rotas{i}(1,:); % Definindo a posição de inicio     
        goal{i} = rotas{i}(2,:); % Definindo a posição do alvo        
        
        % Estado inicial do robô 
        %   [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
        x{i} = [start{i}(1) start{i}(2) pi/2 1 100]';        
    
        % Corrigindo o angulo
        x{i}(3) = novoAngulo(x{i},goal{i});        

        % resultado da simulação
        result.x{i} = [];   
        historico_dist_alvo{i}=[];
        rota_finalizada{i} = false;
        waypoints_visitados{i}=[]; % Variável para registrar e colorir os pontos visitados
    end
    
    % Posição dos obstáculos [x(m) y(m)]
    obstacle = Obstaculos;  
    
    % Posição dos obstáculos dinâmicos
    global z;
    z = 10;
    a=0; b=14;    
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
    
    % Movimento principal
    fig = figure(numfig);
    simular=true;

    if gravar        
        video = VideoWriter(nomeVideo); % Criar um objeto VideoWriter para gravar
        video.FrameRate = 30; % Definir taxa de quadros (frames per second)               
        open(video); % Abrir o objeto VideoWriter para escrita
    end

    while simular

        if gravar        
            % Adicionar o frame ao vídeo
            frame_gravado = getframe(fig);
            writeVideo(video, frame_gravado);
        end
    
        % ==== Animação ====                    
        hold off;
        title(titulo)

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

        axis(area);            
        % drawnow;        

        % Movimento dos robôs
        for i = 1:Nv
        
            % DWA - Calculando valores de entrada
            [u, traj] = DynamicWindowApproach(x{i}, Kinematic, goal{i}, evalParam, obstacle, obstacleR);
            
            % Locomoção por modelo de movimento
            x{i} = f(x{i}, u);
            
            % Salvando resultados de simulação
            result.x{i} = [result.x{i}; x{i}'];

            %Verificando se ele está se distanciando do objetivo
            dist_goal = norm(goal{i} - x{i}(1:2)); % colentando a distância e salvando no histórico
            historico_dist_alvo{i}= [historico_dist_alvo{i}; dist_goal];
                        
            % verificando as 2 últimas distâncias
            if (size(historico_dist_alvo{i},1)>2)
                distancias = historico_dist_alvo{i}(end-2:end);                
                if (distancias == sort(distancias))==true	                 
                    % Corrigindo o angulo novamente
                    x{i}(3) = novoAngulo(x{i},goal{i});       
                end
            end

            
            % Julgamento de meta
            if norm(x{i}(1:2) - goal{i}') < 0.2
                disp('Chegou ao objetivo!!');    
                
                % Verifica se é um waypoint válido para registrar
                % if ismember(goal{i}, WayPoints, 'rows')
                if ismember(goal{i}, wp_visitados{i}, 'rows')
                    waypoints_visitados{i}=[waypoints_visitados{i}; goal{i}]; 
                end

                % Verifica se já visitou todas as rotas
                if length(rotas{i})>2
                    rotas{i}(1,:)=[]; %apagando a primeira linha                

                    start{i} = rotas{i}(1,:); % Definindo a posição de inicio     
                    goal{i} = rotas{i}(2,:); % Definindo a posição do alvo        
                    
                    % Estado inicial do robô 
                    %   [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
                    x{i} = [start{i}(1) start{i}(2) pi/2 1 100]';        
                
                    % Corrigindo o angulo
                    x{i}(3) = novoAngulo(x{i},goal{i});    
                   
                else
                    rota_finalizada{i} = true;
                    disp('Fim de uma rota');                    
                end
                if isequal(rota_finalizada{1:Nv})&&rota_finalizada{i}==true
                    clc;
                    disp('Todos os VANTs completaram a rota');  
                    % if gravar    
                    %     close(video); % Fechar o objeto VideoWriter/gravação
                    % end
                    simular = false;
                    break;                      
                end    
            end            
            
    
            % Objetivo
            % plot(goal{i}(1), goal{i}(2), '*g');                

            % Plotando os pontos visitados
            if ~(isempty(waypoints_visitados{i}))
                plot(waypoints_visitados{i}(:,1),waypoints_visitados{i}(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',cor(i));
            end
    
            % comprimento da seta
            ArrowLength = 0.3;
            
            % robô
            quiver(x{i}(1), x{i}(2), ArrowLength*cos(x{i}(3)), ArrowLength*sin(x{i}(3)), 'ok');
            hold on;
            % plot(result.x{i}(:,1), result.x{i}(:,2), '-.',Color=cor(i)); % caminho inteiro
            
            partes=40;
            if size(result.x{i},1)>partes
                plot(result.x{i}(end-partes:end,1), result.x{i}(end-partes:end,2), '-.',Color=cor(i)); % caminho parcial
            else
                plot(result.x{i}(:,1), result.x{i}(:,2), '-.',Color=cor(i)); % caminho inteiro
            end
            hold on;
            
            % Explore a representação da trajetória
            if ~isempty(traj)
                for it = 1:length(traj(:,1))/5
                    ind = 1 + (it-1)*5;
                    % plot(traj(ind,:), traj(ind+1,:), '-g');
                    plot(traj(ind,:), traj(ind+1,:), '-m');
                    hold on;
                end
            end            
            
            
            % Obstáculos dinâmicos 
            plot(obst_Din(:,1), obst_Din(:,2), '*k');
            hold on;

            a=-0.02; b=0.02;
            deslocamento = a + (b-a).*rand(z,2);
            obst_Din = obst_Din + deslocamento;

            % Atualizando os obstáculos
            obstacle = [Obstaculos;obst_Din] ;  
            
        end
        drawnow;
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




function novo_angulo = novoAngulo(x,goal)
    % Corrigindo o angulo
    % Definir os pontos A e B
    A = [x(1) x(2)];
    B = goal;
    
    % Calcular o ângulo entre os pontos A e B em radianos
    novo_angulo = atan2(B(2) - A(2), B(1) - A(1));    
end
