clc; clear all; close all;

% Forçando que o YALMIP seja instalado, se mudar de PC tem que mudar o diretório
addpath(genpath("C:\Program Files\MATLAB\R2021b\toolbox\YALMIP-master"))

% Define o número de_ cidades e número de vendedores
num_caixeiros = 8; % número de vendedores

% Galpão inicial e final
figure(1);
hold on
%grid on
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
num_cidades = size(allPoints,1);

% Gerando pesos aleatórios para cada Waypoint
A = 1; % Valor mínimo
B = 4; % Valor máximo
%randVec = randi([A,B],1,num_cidades-2);
%Pesos = [0,randVec,0]; % 0 no inicio e no final pq são os galpoes
Pesos = [0     4     3     3     2     1     3     1     1     3     1     4     4     3     1     0];
%Pesos = ones(1,16); Pesos(1) = 0; Pesos(16)=0;
randVec = Pesos(:,(2:num_cidades-1));
text(x+0.2,y-0.2,"Peso:" + num2str(randVec'))

%% Define a matriz de distância entre os pontos
%dist = squareform(pdist(allPoints)); % pela distância euclidiana

% pela distância do Astar
%load("valor_dist.mat","dist"); % Para evitar processamento desnecessário nos testes já salveis as distâncias do Astar

dist = zeros(size(allPoints,1), size(allPoints,1));
for i=1:size(allPoints,1)
    ponto_inicial = allPoints(i,:);
    for j=1:size(allPoints,1)
        if (i~=j)&&(dist(i,j)==0) %% Para não fazer o calculo duas vezes, já que dist(i,j)==dist(j,i), fazendo 1 vez já calcula as duas variáveis
            ponto_final = allPoints(j,:);
            dist(i,j)=A_Star_ver2(Obstaculos,ponto_inicial,ponto_final);
            dist(j,i)=dist(i,j);
        end
    end
end



%% RESTRIÇÕES LUCAS

% Velocidade media (m/s) da aeronave
VelocidadeMedia = 0.43;

% Calculo do tempo necessario  entre cada waypoints
TempoDistWay = dist./VelocidadeMedia;

% Tempo de execucao disponivel na bateria
%TempoDisponivelBateria=50;
TempoDisponivelBateria=[42    62    47    65    66    69    55    38    45    77    42    72];

% Capacidade dos drones
Capacidade = [8     5     6    14    12     8    15     5     9     9    13    13];

% Peso para a maximizacao do peso total para o custo da trajetoria [GamaD]
CostWeight = 40;

% Peso para a a minimizacao da distancia percorrida [AlphaC]
CostDistance = 1;

%% Define as variáveis binárias x(i,j,k) que indicam se a cidade i é visitada antes da cidade j pelo vendedor k
x = binvar(num_cidades, num_cidades, num_caixeiros, 'full');

% Define a função objetivo como a soma das distâncias percorridas por todos os vendedores
%Objective = (sum(CostDistance*sum(sum(DistWay.*xWP))) - CostWeight*CostWay*sum(xWP,2));
obj = 0;
for k = 1:num_caixeiros
    %obj = obj + sum(sum(dist.*(x(:,:,k))));
    %obj = obj + sum(sum(dist.*(x(:,:,k))))*CostDistance;
    %obj = obj - CostWeight*Pesos*sum(x(:,:,k),2);
    obj = obj + sum(sum(dist.*(x(:,:,k))))*CostDistance - CostWeight*Pesos*sum(x(:,:,k),2);
end


%% RESTRIÇÕES PARA O SOLVER
for i=1:num_cidades
    for j=1:num_cidades
        if ((i==1) || (j==1))
            U(i,j) = -1;
        else
            U(i,j) = i - j;
        end
    end
end

constraints = []; % restrições
for k=1:num_caixeiros
    % Restrição de inicio, todo caixeiro sai do galpão 1
    constraints = [constraints, sum(x(1,:,k))==1]; %ok

    % Restrição para que ele não vá do galpão inicial direto para o
    % final
    constraints = [constraints, x(1,num_cidades,k)==0]; %ok

    % Restrição de retorno, todo caixeiro volta para o galpão fim
    constraints = [constraints, sum(x(:,num_cidades,k))==1];

    %Restrição da diagonal principal
    constraints = [constraints, trace(x(:,:,k))==0];

    %Forcar a nao voltar para o primeiro WayPoint
    constraints = [constraints, sum(x(:,1,k)) == 0];

    %Forçar que ele não saia do ultimo WayPoint
    constraints = [constraints, sum(x(num_cidades,:,k)) == 0];

    % Evitar que haja um caminho nao fechado - Soma das colunas deve ser igual a soma das linhas para que seja um percurso fechado viavel - Retirar a 1a. e a ultima coluna para evitar que o incio seja igual ao fim
    constraints = [constraints, sum(x(:,(2:num_cidades-1),k),1) == sum(x((2:num_cidades-1),:,k),2)'];

    % Evitar caminhos intermediarios
    %constraints = [constraints, U + ones(num_cidades) <= (ones(num_cidades) - x(:,:,k))*num_cidades];
    %constraints = [constraints, gerar_rota(x(:,:,k),num_cidades)==false];


    % Tempo total da rota deve ser menor que o TempoDisponivelBateria
    constraints = [constraints, sum(sum(TempoDistWay.*x(:,:,k))) <= TempoDisponivelBateria(k)];

    %Restrição de Capacidade
    constraints = [constraints, sum(sum(x(:,:,k)).*Pesos) <= Capacidade(k)];
end


for k=1:num_caixeiros
    for i=2:(num_cidades-1)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
        constraints = [constraints, sum(x(i,:,k))<=1]; %pegando linha a linha
    end
end

for i=1:(num_cidades)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
    for j=1:num_cidades     %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
        constraints = [constraints, sum(x(i,j,:))<=1]; %pegando um por um
    end
end

% %Forçar que todas as cidades sejam visitadas
% for i=2:(num_cidades-1)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
%     constraints = [constraints, sum(sum(x(i,:,:)))==1]; %pegando linha a linha
% end

%Contrário da restrição acima, não vai visitar todas por conta da bateria
for i=2:(num_cidades-1)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
    constraints = [constraints, sum(sum(x(i,:,:)))<=1]; %pegando linha a linha
end

% %Forçar que todas as cidades sejam visitadas
% for j=2:(num_cidades-1)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
%     constraints = [constraints, sum(sum(x(:,j,:)))==1]; %pegando linha a linha
% end

%Contrário da restrição acima, não vai visitar todas por conta da bateria
for j=2:(num_cidades-1)  %Para não pegar o primeiro e nem o ultimo, vai de 2 até n-1
    constraints = [constraints, sum(sum(x(:,j,:)))<=1]; %pegando linha a linha
end

%Restrição de que cada vendedor visita apenas cidades conectadas
for k = 1:num_caixeiros
    for i = 1:num_cidades-1
        for j = 1:num_cidades-1
            constraints = [constraints, x(i,j,k) + x(j,i,k) <= 1];
        end
    end
end

% teste
% for k = 1:num_caixeiros
%     for i = 2:num_cidades-1
%         for j = 2:num_cidades-1
%             if i~=j
%                 constraints = [constraints, (i-j+1) <= (1-x(i,j,k))*(num_cidades-1)];
%             end
%         end
%     end
% end

% Define as opções do solver
ops = sdpsettings('verbose', 1); %A opção verbose é usada para definir o nível de detalhe da saída do solver. Quando verbose é definido como 1, o solver produz saídas detalhadas do processo de solução. Quando verbose é definido como 0, o solver produz uma saída mais sucinta.


% Resolve o problema
sol = optimize(constraints, obj, ops);
%sol = solvesdp(constraints,obj);

% Exportar resultados da Resultado da Otimizacao para o Workspace
assignin('caller', 'x', double(x));

% Exibe a solução
if sol.problem == 0
    disp('Solução encontrada');
    x_val = value(x);
    tempo_total=0;
    dist_total = 0;
    tempo_robo = [];
    peso_robo = [];
    for k = 1:num_caixeiros
        disp(" ");
        disp("-------------------");
        disp("Caixeiro " + num2str(k) + "| distância: "+ num2str(sum(sum(dist.*x(:,:,k))))+"m");
        disp("-------------------");
        dist_total = dist_total + sum(sum(dist.*x(:,:,k)));
        tempo_total = tempo_total + sum(sum(TempoDistWay.*x(:,:,k)));
        tempo_robo = [tempo_robo ;sum(sum(TempoDistWay.*x(:,:,k)))];
        peso_robo = [peso_robo;sum(Pesos.*sum(x(:,:,k)))];
        % Definindo a cor do plot para cada caixeiro
        switch k
            case 1
                ColorPlot = [0 0 0]; %Preto
            case 2
                ColorPlot = [1 0 0]; %RED
            case 3
                ColorPlot = [0 0 1]; %BLUE
            case 4
                ColorPlot = [0 1 0]; %VERDE
            case 5
                ColorPlot = [1 0 1]; %MAGENTA
            case 6
                ColorPlot = [0 1 1]; %cyan
            case 7
                ColorPlot = [0.8500 0.3250 0.0980]; %
            case 8
                ColorPlot = [0.9290 0.6940 0.1250]; %
            otherwise
                ColorPlot = rand(1,3);
        end


        disp(['Caminho do caixeiro ', num2str(k), ':']);

        %rota_ordenada = gerar_rota(x(:,:,k),num_cidades);


        for i = 1:num_cidades
            for j = 1:num_cidades
                %if x_val(i,j,k) == 1
                if x_val(i,j,k) >0.5
                    disp(['De ', num2str(i), ' para ', num2str(j)]);
                    %Adicionei para plotar
                    plotway = [allPoints(i,:);allPoints(j,:)];

                    fig = figure(1);
                    set(fig, 'WindowState', 'maximized')

                    %plot(plotway(:,1), plotway(:,2), 'LineStyle', '--', 'Color', ColorPlot);

                    %Deixar o waypoint colorido quando for coletado
                    if ((j~=1)&&(j~=num_cidades))
                        plot(allPoints(j,1),allPoints(j,2),'square','MarkerSize' ,20, 'MarkerEdgeColor' , [0,0,0], 'MarkerEdgeColor','k','MarkerFaceColor',ColorPlot);
                    end

                    %Plotar pelo Astar
                    Optimal_path = A_Star_plot(Obstaculos,allPoints(i,:),allPoints(j,:),ColorPlot);
                end
            end
        end

    end
    %% MOSTRANDO OS RESULTADOS
    disp(" ")
    disp("Distância total realizada pelos caixeiros :"+ num2str(dist_total)+"m");
    disp("Tempo total para realizar o percurso :"+ num2str(tempo_total)+"s");
    disp("Tempo minimo:"+ num2str(min(tempo_robo))+"s");
    disp("Tempo máximo:"+ num2str(max(tempo_robo))+"s");
    disp("Pontos coletados:"+ sum(peso_robo));

    %% SALVANDO AS FIGURAS AUTOMATICAMENTE
%     nome_do_arquivo = num2str(num_caixeiros)+"_robos";
%     savefig("C:\Users\Notebook\Desktop\Artigo\Figuras\Experimento 4/"+nome_do_arquivo+".fig")
%     saveas(gcf, "C:\Users\Notebook\Desktop\Artigo\Figuras\Experimento 4/"+nome_do_arquivo, "png")

else
    disp('Problema não resolvido');
end




