%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Disciplina: Estudos Especiais - Mestrado UFC 31/10/2023

clc; clear; close all;

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("funcoes_auxiliares\")
addpath("AStar\")

%% CARREGANDO O MAPA

% Galpão inicial e final
figure(1);
hold on

galpaoIncicial = [1,1];
plot(galpaoIncicial(1),galpaoIncicial(2),'b--o','MarkerSize' ,10);
text(galpaoIncicial(1)+0.2,galpaoIncicial(2)-0.2,'Galpao Inicial')

galpaoFinal = [13,9];
plot(galpaoFinal(1),galpaoFinal(2),'r--o','MarkerSize' ,10);
text(galpaoFinal(1)+0.2,galpaoFinal(2)+0.2,'Galpao Final')

% waypoints
WayPoints = [6,1; 8,9; 4,8; 13,6; 2,9; 1,4; 3,4; 11,7;5,3;4,6;10,2;7,5;8,4;10,6];
plot(WayPoints(:,1),WayPoints(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor','y');
x = WayPoints(:,1);
y = WayPoints(:,2);
z = (2:1:size(WayPoints,1)+1)';
text(x+0.2,y,"WP:"+ num2str(z))

%Obstaculos
Obstaculos = [];
Obstaculos = [Obstaculos; criar_obstaculo(4,9,11,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(6,5,5,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(8,9,10,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(8,5,6,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(9,3,4,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(7,2,4,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(4,6,7,0.01)];
Obstaculos = [Obstaculos; criar_obstaculo(8,12,13,0.01)];
plot(Obstaculos(:,1),Obstaculos(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0.5, 0.5, 0.5],'MarkerFaceColor',[0.5, 0.5, 0.5]);


xlim([(min(WayPoints(:,1))-1.5) (max(WayPoints(:,1))+1.5)])
ylim([(min(WayPoints(:,2))-1.5) (max(WayPoints(:,2))+1.5)])

% Pontos
allPoints = [galpaoIncicial;WayPoints;galpaoFinal];
x = allPoints(:,1);
y = allPoints(:,2);
num_cidades = size(allPoints,1);


%% Plotando o mapa
% figure(1)
% plotarmapa(x,y)
% plot_obstaculos(Obstaculos)
% hold on

%% Calculando a matriz de distâncias pelo Astar
gerar_matriz = false;


if gerar_matriz
    dim = length(x);
    Djk = zeros(dim);
    for i=1:dim
        ponto_inicial = [x(i),y(i)];
        for j=1:dim
            if (i~=j)&&(Djk(i,j)==0) %% Para não fazer o calculo duas vezes, já que dist(i,j)==dist(j,i), fazendo 1 vez já calcula as duas variáveis
                ponto_final = [x(j),y(j)];
                Djk(i,j)=A_Star_ver2(Obstaculos,ponto_inicial,ponto_final,30,30);
                Djk(j,i)=Djk(i,j);
            end
        end
    end
    save("Djk_Astar.mat","Djk")
else
    load("Djk_Astar.mat","Djk")
end

%% =====================================
% EXPERIMENTOS
% =====================================
exp_max = 20;
V_NV = [8];
for aux=1:1
    Nv = V_NV(aux);
    minhaString = [];
    tic;
    for experimento=1:exp_max
        %% PARÂMETROS COLÔNIA DE FORMIGAS
        % Nv = 4; % Número de veículos==formigas
        Ng = 24; % Número de grupos
        
        alpha = 1;
        beta = 2;
        rho = 0.5;
        
        q0 = 0.9; % probabilidade de selecionar a formiga com custo parcial mínimo
        q1 = 0.05; % probabilidade de selecionar a formiga com custo parcial máximo
        
        p0 = 0.9; % probabilidade de escolher as tarefas com maior informação heurística
        
        it_max = 10000;
        
        
        tij = 0; % Tempo de subida
        vij = 0.43; % velocidade em m/s
        
        % cjk = Djk./vij + (ones(length(Djk)) - eye(length(Djk))).*tij; % Custo para ir do ponto j até o ponto k
        cjk = Djk; %Alterando o custo para distância percorrida
        
        nrs = 1./cjk;
        nrs(nrs==inf) = 0;
        lambda = Nv/2; % Principio da frente de pareto f1>lambda*f2


        %% ETAPA 1: Inicialização
    
        solucao_valida = false;
        while ~solucao_valida
    
            C = (2:1:length(Djk)-1); % Tarefas não atribuídas
    
            for i=1:Nv
                P{i} = 1; % Caminhos realizados por cada veículo
                xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
            end
    
            while ~isempty(C)
                formiga_i = randi(Nv); % Seleciona uma formiga aleatória
                r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
                [s, s_index] = argMin(cjk,r,C); % Encontra a próxima tarefa
                P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î
                C(s_index) = []; % Remove s de C;
            end
    
            for i=1:Nv
                P{i} = [P{i} length(Djk)]; % Adicionando o retorno ao depósito (galpão)
            end
    
            P_best = P; % Salva como o melhor resultado
    
            % Gera a matriz de informação heurística de cada formiga e atualiza a matriz xijk
            Cp = zeros(1,Nv); % Custo parcial de cada formiga
            for i=1:Nv
                for j=1:length(P{i})-1
                    r = P{i}(j);
                    s = P{i}(j+1);
                    xijk{i}(r,s) = 1;
                end
    
                Cp(i) = custoParcial(xijk{i},cjk);  % Custo parcial de cada formiga
    
            end
    
            f1 = sum(Cp); % Custo global f1
            f2 = max(Cp); % Valor máximo de f2
    
            % Verifica se a solução é válida
            if f1>=lambda*f2
                solucao_valida = true;
                P_best = P;
                Cp_best = Cp;
            end
    
        end
    
        % Inicializando as matrizes de feromônios zeradas
        trs1 = zeros(length(Djk));
        trs2 = zeros(length(Djk));
    
        % Atualizando as matrizes de feromônio iniciais
        for r=1:length(Djk)
            for s=1:length(Djk)
                if xijk{i}(r,s)==1 % verifica se há ligação entre r e s para atualizar o feromônio
                    trs1(r,s) = trs1(r,s) + 1./(xijk{i}(r,s)*cjk(r,s));
                    trs2(r,s) = trs2(r,s) + 1/Nv./(xijk{i}(r,s)*cjk(r,s));
                end
            end
        end
    
        % Melhores feromônios
        trs1o = trs1;
        trs2o = trs2;
    
        % Plotando o caminho gerado pelos VANTs
%         drawnow
%         plotando_caminhos(P_best,Nv,x,y,Obstaculos)
%         pause(2)
%         hold off
    
        %% ETAPA 2: Construção de soluções
    
        resultados = zeros(it_max,3);
        for it=1:it_max
            resultados(it,1) = it;
            resultados(it,2) = sum(Cp_best);
            resultados(it,3) = max(Cp_best);
    
            clc;
            disp("Iteração: "+num2str(it));
    
            for j=1:Ng
    
                solucao_valida = false;
                while ~solucao_valida
    
                    C = (2:1:length(Djk)-1); % Tarefas não atribuídas
    
                    for i=1:Nv
                        P{i} = 1; % Caminhos realizados por cada veículo
                        xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
                    end
    
                    while ~isempty(C)
                        formiga_i = selecionar_formiga(P,cjk,q0,q1); % Seleciona uma formiga com base nos critérios min e max
                        r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
                        [s,s_index] = argMax(trs1,trs2,nrs,r,C,alpha,beta,p0); % Encontra a próxima tarefa
                        P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î
                        [trs1,trs2] = att_Feromonio(trs1o,trs2o,trs1,trs2,rho,r,s); % Atualização do feromônio do caminho rs
                        C(s_index) = []; % Remove s de C;
                    end
    
                    Cp = zeros(1,Nv); % Custo parcial de cada formiga
                    for i=1:Nv
                        P{i} = [P{i} length(Djk)]; % Adicionando o retorno ao depósito (galpão)
    
                        % Gerando a matriz de variáveis de decisão
                        for j=1:length(P{i})-1
                            r = P{i}(j);
                            s = P{i}(j+1);
                            xijk{i}(r,s) = 1;
                        end
    
                        Cp(i) = custoParcial(xijk{i},cjk);  % Custo parcial de cada formiga
                    end
    
                    f1 = sum(Cp); % Custo global f1
                    f2 = max(Cp); % Valor máximo de f2
    
                    % Verifica se a solução é válida
                    if f1>=lambda*f2
                        solucao_valida = true;
                        if f1<=sum(Cp_best)
                            %Salvando os melhores resultados
                            P_best = P;
                            Cp_best = Cp;
    
                            % Melhores feromônios
                            trs1o = trs1;
                            trs2o = trs2;
    
                        end
                    end
    
                end
    
            end
    
            % ETAPA 3: Atualização global do feromônio
            [trs1,trs2] = att_Feromonio_Global(trs1o,trs2o,trs1,trs2,rho,f1,f2);
    
        end
    
        %% PLOTANDO RESULTADOS
    
        % Plotando o mapa
        hold off
        % figure(1)
        % plotarmapa(x,y)
        % plot_obstaculos(Obstaculos)
    
        % Galpão inicial e final
        figure(1);
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
        x = WayPoints(:,1);
        y = WayPoints(:,2);
        z = (2:1:size(WayPoints,1)+1)';
        text(x+0.2,y,"WP:"+ num2str(z))
    
        %Obstaculos
        Obstaculos = [];
        Obstaculos = [Obstaculos; criar_obstaculo(4,9,11,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(6,5,5,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(8,9,10,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(8,5,6,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(9,3,4,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(7,2,4,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(4,6,7,0.01)];
        Obstaculos = [Obstaculos; criar_obstaculo(8,12,13,0.01)];
        plot(Obstaculos(:,1),Obstaculos(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0.5, 0.5, 0.5],'MarkerFaceColor',[0.5, 0.5, 0.5]);
    
    
        xlim([(min(WayPoints(:,1))-1.5) (max(WayPoints(:,1))+1.5)])
        ylim([(min(WayPoints(:,2))-1.5) (max(WayPoints(:,2))+1.5)])
    
        % Pontos
        allPoints = [galpaoIncicial;WayPoints;galpaoFinal];
        x = allPoints(:,1);
        y = allPoints(:,2);
        num_cidades = size(allPoints,1);
    
        drawnow
        plotando_caminhos(P_best,Nv,x,y,Obstaculos)

        % tempo de processamento
        tempoDecorrido = toc;
        
        % Salvando a figura
        nomeDoArquivoFig = ["Experimento_"+num2str(experimento)+"-"+num2str(Nv)+"VANTS.fig"];
        savefig(nomeDoArquivoFig);
        
        % Salvar todas as variáveis em um arquivo .mat
        nomeDoArquivoMat = ["Experimento_"+num2str(experimento)+"-"+num2str(Nv)+"VANTS.mat"];
        save(nomeDoArquivoMat);

    
%         figure(2)
%         plot(resultados(:,1),resultados(:,2),'','Color','k')
%         hold on
%         plot(resultados(:,1),resultados(:,3),'','Color','r')
%         title('Custo gerado')
%         xlabel('Iterações')
%         ylabel('Distância(m)')
%         legend('Somatório do tempo de voo de todos os VANTs','Tempo máximo da missão','Location', 'southoutside')
    
        %% Resultados em texto
        Dt_m = sum(Cp_best);
        Tmax_seg = max(Cp_best)/vij;
       
        minhaString = [minhaString; "Experimento "+num2str(experimento)];
        minhaString = [minhaString;"Dist(m): "+num2str(Dt_m)];
        minhaString = [minhaString;"Tmax(s): "+num2str(Tmax_seg)];
        minhaString = [minhaString;"Tempo de CPU(s): "+ num2str(tempoDecorrido)];

        minhaString = [minhaString; " "];
        minhaString = [minhaString; " "];
    
        % Tenho que salvar a figura do melhor experimento
        % Pegar a média/ desv.pad.. max e min;
    
        
        % Fechando as figuras
        close all;
    end

    %% Salvando o txt dos resultados
    nomeArquivo = ["Resultados_"+num2str(Nv)+"_VANTs.txt"];
    salvarStringNoArquivo(minhaString, nomeArquivo);

end

