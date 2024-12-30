%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Algoritmo ACO puro para parametrização

clc; clear; close all;

vet_param = [0.1,0.2,0.3];

for num_param = 1:length(vet_param)    
    q1 = vet_param(num_param);
    
    for simu=1:200
        nome_arq = "resultado_parametrizacao/q1/simu_"+simu+"_q1_"+q1+".mat";

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("funcoes_auxiliares\")

%% CARREGANDO O MAPA

% Galpão inicial e final
figure(1);
hold on

galpaoIncicial = [1,1];
plot(galpaoIncicial(1),galpaoIncicial(2),'b--o','MarkerSize' ,10);
text(galpaoIncicial(1)+0.2,galpaoIncicial(2)-0.2,'Galpao Inicial')

% galpaoFinal = [13,9];
% plot(galpaoFinal(1),galpaoFinal(2),'r--o','MarkerSize' ,10);
% text(galpaoFinal(1)+0.2,galpaoFinal(2)+0.2,'Galpao Final')

% waypoints
WayPoints = [6,1; 8,9; 4,8; 13,6; 2,9; 1,4; 3,4; 11,7;5,3;4,6;10,2;7,5;8,4;10,6];
plot(WayPoints(:,1),WayPoints(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor','y');
x = WayPoints(:,1);
y = WayPoints(:,2);
z = (2:1:size(WayPoints,1)+1)';
text(x+0.2,y,"WP:"+ num2str(z))

xlim([(min(WayPoints(:,1))-1.5) (max(WayPoints(:,1))+1.5)])
ylim([(min(WayPoints(:,2))-1.5) (max(WayPoints(:,2))+1.5)])

% Pontos
% allPoints = [galpaoIncicial;WayPoints;galpaoFinal];
allPoints = [galpaoIncicial;WayPoints;galpaoIncicial]; % Definindo o galpão inicial igual ao final
x = allPoints(:,1);
y = allPoints(:,2);
num_cidades = size(allPoints,1);


%% Calculando a matriz de distâncias pela distância euclidiana
cjk = pdist2(allPoints, allPoints); % Calcular a matriz de distâncias usando pdist2

%% PARÂMETROS COLÔNIA DE FORMIGAS
Nv = 3; % Número de veículos==formigas
Ng = 100; % Número de grupos

alpha = 0.5;
beta = 4;
rho = 0.25;

q0 = 0.4; % probabilidade de selecionar a formiga com custo parcial mínimo
% q1 = 0.05; % probabilidade de selecionar a formiga com custo parcial máximo

p0 = 0.9; % probabilidade de escolher as tarefas com maior informação heurística
% % % % % 
it_max = 1000;

tij = 0; % Tempo de subida
vij = 0.43; % velocidade em m/s

nrs = 1./cjk;
nrs(nrs==inf) = 0;
lambda = Nv/2; % Principio da frente de pareto f1>lambda*f2


%% ETAPA 1: Inicialização

solucao_valida = false;
while ~solucao_valida

    C = (2:1:length(cjk)-1); % Tarefas não atribuídas

    for i=1:Nv
        P{i} = 1; % Caminhos realizados por cada veículo
        xijk{i} = zeros(length(cjk)); % variável de decisão binária [formiga î foi de j até k]
    end

    while ~isempty(C)
        formiga_i = randi(Nv); % Seleciona uma formiga aleatória
        r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
        [s, s_index] = argMin(cjk,r,C); % Encontra a próxima tarefa
        P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î
        C(s_index) = []; % Remove s de C;
    end

    for i=1:Nv
        P{i} = [P{i} length(cjk)]; % Adicionando o retorno ao depósito (galpão)
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
trs1 = zeros(length(cjk));
trs2 = zeros(length(cjk));

% Atualizando as matrizes de feromônio iniciais
for r=1:length(cjk)
    for s=1:length(cjk)
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
drawnow
numfig = 1;
plotando_caminhos_aco_puro(P,Nv,x,y,numfig)
% pause(2)
hold off

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

            C = (2:1:length(cjk)-1); % Tarefas não atribuídas

            for i=1:Nv
                P{i} = 1; % Caminhos realizados por cada veículo
                xijk{i} = zeros(length(cjk)); % variável de decisão binária [formiga î foi de j até k]
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
                P{i} = [P{i} length(cjk)]; % Adicionando o retorno ao depósito (galpão)

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
    [trs1,trs2] = att_Feromonio_Global(trs1o,trs2o,trs1,trs2,rho,f1,f2,Nv,P);
    

end

%% PLOTANDO RESULTADOS

% Plotando o mapa
hold off

% Galpão inicial e final
figure(1);
galpaoIncicial = [1,1];
plot(galpaoIncicial(1),galpaoIncicial(2),'b--o','MarkerSize' ,10);
text(galpaoIncicial(1)+0.2,galpaoIncicial(2)-0.2,'Galpao Inicial')
hold on

% galpaoFinal = [13,9];
% plot(galpaoFinal(1),galpaoFinal(2),'r--o','MarkerSize' ,10);
% text(galpaoFinal(1)+0.2,galpaoFinal(2)+0.2,'Galpao Final')

% waypoints
WayPoints = [6,1; 8,9; 4,8; 13,6; 2,9; 1,4; 3,4; 11,7;5,3;4,6;10,2;7,5;8,4;10,6];
plot(WayPoints(:,1),WayPoints(:,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor','y');
x = WayPoints(:,1);
y = WayPoints(:,2);
z = (2:1:size(WayPoints,1)+1)';
text(x+0.2,y,"WP:"+ num2str(z))

xlim([(min(WayPoints(:,1))-1.5) (max(WayPoints(:,1))+1.5)])
ylim([(min(WayPoints(:,2))-1.5) (max(WayPoints(:,2))+1.5)])

% Pontos
allPoints = [galpaoIncicial;WayPoints;galpaoIncicial];
x = allPoints(:,1);
y = allPoints(:,2);
num_cidades = size(allPoints,1);

drawnow
title("Melhor Resultao ACO*")
numfig = 1;
plotando_caminhos_aco_puro(P_best,Nv,x,y,numfig)



%% Salvando as variáveis e a melhor figura
save(nome_arq);

%% fim da simulação
clc; close all;

    end
end
