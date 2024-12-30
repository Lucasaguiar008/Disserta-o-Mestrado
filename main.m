%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Disciplina: Estudos Especiais - Mestrado UFC 31/10/2023

clc; clear; close all;

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("funcoes_auxiliares\")

%% CARREGANDO O MAPA

% Atenção, dependendo do mapa é necessário alterar a função Ler_mapa 
% para mudar a quantidade de linhas que ele deve pegar
[x,y] = Ler_mapa('mapas_caixeiros/bays29.tsp',38); 

% Cria uma matriz de distâncias d usando a função pdist2
% calcula as distâncias euclidianas entre todos os pares de pontos.
Djk = pdist2([x,y], [x,y]);


%% PARÂMETROS COLÔNIA DE FORMIGAS
Nv = 3; % Número de veículos==formigas
Ng = 24; % Número de grupos

alpha = 1;
beta = 2;
rho = 0.5;

q0 = 0.9; % probabilidade de selecionar a formiga com custo parcial mínimo
q1 = 0.05; % probabilidade de selecionar a formiga com custo parcial máximo

p0 = 0.9; % probabilidade de escolher as tarefas com maior informação heurística

it_max = 10000;

tij = 10; % Tempo de subida
vij = 20; % velocidade em m/s

cjk = Djk./vij + (ones(length(Djk)) - eye(length(Djk))).*tij; % Custo para ir do ponto j até o ponto k
nrs = 1./cjk;
nrs(nrs==inf) = 0;
lambda = Nv/2; % Principio da frente de pareto f1>lambda*f2


%% ETAPA 1: Inicialização

solucao_valida = false;
while ~solucao_valida

    C = (2:1:length(Djk)); % Tarefas não atribuídas
    
    for i=1:Nv
       P{i} = 1; % Caminhos realizados por cada veículo 
       xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
%        nrs{i}  = zeros(length(Djk)); % informação heurística da formiga î
    end
    
    while ~isempty(C)
        formiga_i = randi(Nv); % Seleciona uma formiga aleatória
        r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
        [s, s_index] = argMin(cjk,r,C); % Encontra a próxima tarefa
        P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î
        C(s_index) = []; % Remove s de C;
    end
    
    for i=1:Nv
       P{i} = [P{i} 1]; % Adicionando o retorno ao depósito (galpão 1)
    end
    
    P_best = P; % Salva como o melhor resultado
    
    % Gera a matriz de informação heurística de cada formiga e atualiza a matriz xijk
    Cp = zeros(1,Nv); % Custo parcial de cada formiga
    for i=1:Nv
        for j=1:length(P{i})-1
            r = P{i}(j);
            s = P{i}(j+1);
%             nrs{i}(r,s) = 1./cjk(r,s);
    
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
trs2o= trs2;

% PLOTANDO RESULTADOS
figure(2)
plot_caminhos(P_best,x,y,Nv)

%% ETAPA 2: Construção de soluções

resultados = zeros(it_max,3);
for it=1:it_max
    resultados(it,1) = it;
    resultados(it,2) = sum(Cp_best);
    resultados(it,3) = max(Cp_best);

    figure(2)
    title("Melhor caminho dos "+num2str(Nv)+" VANTs. | it: "+num2str(it))

    for j=1:Ng
    
        solucao_valida = false;
        while ~solucao_valida
            
            C = (2:1:length(Djk)); % Tarefas não atribuídas
    
            for i=1:Nv
                P{i} = 1; % Caminhos realizados por cada veículo
                xijk{i} = zeros(length(Djk)); % variável de decisão binária [formiga î foi de j até k]
            end
    
            while ~isempty(C)
                formiga_i = selecionar_formiga(P,cjk,q0,q1); % Seleciona uma formiga com base nos critérios min e max
                r = P{formiga_i}(end); % última tarefa atribuída a formiga î (no caso o galpão)
%                 [s,s_index] = argMax(trs1,trs2,nrs{formiga_i},r,C,alpha,beta,p0); % Encontra a próxima tarefa
                [s,s_index] = argMax(trs1,trs2,nrs,r,C,alpha,beta,p0); % Encontra a próxima tarefa
                P{formiga_i} = [P{formiga_i} s]; % Atribuindo a tarefa s para a formiga î
                [trs1,trs2] = att_Feromonio(trs1o,trs2o,trs1,trs2,rho,r,s); % Atualização do feromônio do caminho rs
                C(s_index) = []; % Remove s de C;
            end
    
            Cp = zeros(1,Nv); % Custo parcial de cada formiga
            for i=1:Nv
                P{i} = [P{i} 1]; % Adicionando o retorno ao depósito (galpão 1)
    
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
                    trs2o= trs2;

                    % PLOTANDO RESULTADOS
                    figure(2)
                    plot_caminhos(P_best,x,y,Nv)                    
                    
                end
            end
    
        end
            
    end
    
    % ETAPA 3: Atualização global do feromônio    
    [trs1,trs2] = att_Feromonio_Global(trs1o,trs2o,trs1,trs2,rho,f1,f2);
    
end

%% PLOTANDO RESULTADOS
figure(1)    
plot(resultados(:,1),resultados(:,2),'','Color','k')   
hold on
plot(resultados(:,1),resultados(:,3),'','Color','r')   
title('Custo gerado')
xlabel('Iterações')
ylabel('Tempo(s)')

legend('Somatório do tempo de voo de todos os VANTs','Tempo máximo da missão','Location', 'southoutside')