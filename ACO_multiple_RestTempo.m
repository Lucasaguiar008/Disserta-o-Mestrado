%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Algoritmo híbrido usando ACO + A* com restrição de tempo de VOO
% Este modelo não tem pesos nos Wp (assumindo todos com peso 1)

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


%% Calculando a matriz de distâncias pelo Astar (Ele usa a distância de Manhatan/City Block)
gerar_matriz = false;

% if gerar_matriz
%     dim = length(x);
%     Djk = zeros(dim);
%     paths{dim}{dim} = []; % Criando a variável para salvar os caminhos, funciona apenas para o A_Star_ver3
%     for i=1:dim
%         ponto_inicial = [x(i),y(i)];
%         for j=1:dim
%             if (i~=j)&&(Djk(i,j)==0) %% Para não fazer o calculo duas vezes, já que dist(i,j)==dist(j,i), fazendo 1 vez já calcula as duas variáveis
%                 ponto_final = [x(j),y(j)];
% %                 Djk(i,j)=A_Star_ver2(Obstaculos,ponto_inicial,ponto_final,30,30);
%                 [Djk(i,j), paths{i}{j}]=A_Star_ver3(Obstaculos,ponto_inicial,ponto_final,30,30);                
%                 Djk(j,i)=Djk(i,j);
%                 paths{j}{i}=flip(paths{i}{j}); % É necessário inverter com o flip, pois é o sentido inverso
%             end
%         end
%     end
%     save("Djk_Astar.mat","Djk")
%     save("paths.mat","paths")
% else
%     load("Djk_Astar.mat","Djk")
%     load("paths.mat","paths")
% end


if gerar_matriz
    dim = length(x);
    Djk = zeros(dim);
    paths{dim}{dim} = []; % Criando a variável para salvar os caminhos, funciona apenas para o A_Star_ver3
    for i=1:dim
        ponto_inicial = [x(i),y(i)];
        for j=1:dim            
            ponto_final = [x(j),y(j)];
            %Djk(i,j)=A_Star_ver2(Obstaculos,ponto_inicial,ponto_final,30,30);
            [Djk(i,j), paths{i}{j}]=A_Star_ver3(Obstaculos,ponto_inicial,ponto_final,30,30);                                       
        end
    end
    save("Djk_Astar.mat","Djk")
    save("paths.mat","paths")
else
    load("Djk_Astar.mat","Djk")
    load("paths.mat","paths")
end


%% PARÂMETROS COLÔNIA DE FORMIGAS
Nv = 2; % Número de veículos==formigas

distMax = 22; % distância máxima percorrida
distMax_voo = distMax.*ones(Nv,1);
atingiu_distMax = false*ones(Nv,1); % registrar que não atingiram a capacidade máxima de voo

Ng = 24; % Número de grupos

alpha = 1;
beta = 2;
rho = 0.5;

q0 = 0.9; % probabilidade de selecionar a formiga com custo parcial mínimo
q1 = 0.05; % probabilidade de selecionar a formiga com custo parcial máximo

p0 = 0.9; % probabilidade de escolher as tarefas com maior informação heurística

it_max = 100;

tij = 0; % Tempo de subida
vij = 0.43; % velocidade em m/s

% cjk = Djk./vij + (ones(length(Djk)) - eye(length(Djk))).*tij; % Custo para ir do ponto j até o ponto k
cjk = Djk; %Alterando o custo para distância percorrida

nrs = 1./cjk;
nrs(nrs==inf) = 0;
lambda = Nv/2; % Principio da frente de pareto f1>lambda*f2


%% ETAPA 1: Inicialização

solucao_valida = false;
historico_s = []; % variável auxiliar para verificar se houve saturação na escolha de s
saturou = false;
validacao_form = true;

while ~solucao_valida

    C = (2:1:length(Djk)-1); % Tarefas não atribuídas
    formigas_disponiveis = (1:1:Nv); % Formigas disponíveis

    for i=1:Nv
        P{i} = 1; % Caminhos realizados por cada veículo
        xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
    end

    while (~isempty(C) && ~isempty(formigas_disponiveis)) %verifica se todos os wp foram visitados ou se tem formigas(vants) disponíveis
        formiga_i = formigas_disponiveis(randi(length(formigas_disponiveis))); % Seleciona uma formiga aleatória dentre as formigas disponíveis
        r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
        [s, s_index] = argMin(cjk,r,C); % Encontra a próxima tarefa       
        P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î

        historico_s = [historico_s s]; %Adiciona a tarefa s selecionada no histórico
        saturou = s_saturou(historico_s,10); % Verifica se houve saturação na escolha de S após as últimas 10 escolhas
        
        % Validação da rota
        if validacao_rota(formiga_i,P,cjk,xijk,distMax_voo)
            C(s_index) = []; % Remove s de C;
        else
            P{formiga_i}(end) = []; % Remove a tarefa s que havia sido atribuída
            
            % Verifica se a formiga atingiu seu limite
            [validacao_form, P, C] = validacao_formiga(formiga_i,P,cjk,xijk,distMax_voo,C,saturou);
            
            if ~validacao_form
                formigas_disponiveis(formigas_disponiveis==formiga_i) = []; % Formiga atingiu seu limite na rota
            end            
        end
        
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
drawnow
plotando_caminhos(P_best,Nv,x,y,Obstaculos)
pause(2)
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
        historico_s = []; % variável auxiliar para verificar se houve saturação na escolha de s
        saturou = false; %variável auxiliar
        validacao_form = true; %variável auxiliar

        while ~solucao_valida

            C = (2:1:length(Djk)-1); % Tarefas não atribuídas
            formigas_disponiveis = (1:1:Nv); % Formigas disponíveis

            for i=1:Nv
                P{i} = 1; % Caminhos realizados por cada veículo
                xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
            end

            while (~isempty(C) && ~isempty(formigas_disponiveis)) %verifica se todos os wp foram visitados ou se tem formigas(vants) disponíveis
                formiga_i = selecionar_formiga_com_rest_tempo(P,cjk,q0,q1,formigas_disponiveis); % Seleciona uma formiga com base nos critérios min e max
                r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
                [s,s_index] = argMax(trs1,trs2,nrs,r,C,alpha,beta,p0); % Encontra a próxima tarefa
                P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î


                historico_s = [historico_s s]; %Adiciona a tarefa s selecionada no histórico
                saturou = s_saturou(historico_s,10); % Verifica se houve saturação na escolha de S após as últimas 10 escolhas
                
                % Validação da rota
                if validacao_rota(formiga_i,P,cjk,xijk,distMax_voo)                    
                    [trs1,trs2] = att_Feromonio(trs1o,trs2o,trs1,trs2,rho,r,s); % Atualização do feromônio do caminho rs
                    C(s_index) = []; % Remove s de C;
                else
                    P{formiga_i}(end) = []; % Remove a tarefa s que havia sido atribuída
                    
                    % Verifica se a formiga atingiu seu limite
                    [validacao_form, P, C] = validacao_formiga(formiga_i,P,cjk,xijk,distMax_voo,C,saturou);
                    
                    % Atualização do feromônio do caminho rs
                    r = P{formiga_i}(end-1);
                    s = P{formiga_i}(end);
                    [trs1,trs2] = att_Feromonio(trs1o,trs2o,trs1,trs2,rho,r,s); 
                    
                    if ~validacao_form
                        formigas_disponiveis(formigas_disponiveis==formiga_i) = []; % Formiga atingiu seu limite na rota
                    end            
                end


                
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
                if f1<=sum(Cp_best)  - pesosUnitarios(P,Nv) % atribuindo pesos sobre os wp
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
title("Resultado ACO+A*")
plotando_caminhos(P_best,Nv,x,y,Obstaculos)


%%
figure(2)
plot(resultados(:,1),resultados(:,2),'','Color','k')
hold on
plot(resultados(:,1),resultados(:,3),'','Color','r')
title('Custo gerado')
xlabel('Iterações')
ylabel('Distância(m)')
legend('Somatório do tempo de voo de todos os VANTs','Tempo máximo da missão','Location', 'southoutside')


