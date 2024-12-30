%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Disciplina: Estudos Especiais - Mestrado UFC 31/10/2023

clc; clear; close all;

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("Resultados\")

%%
num_fig = 1;
vants = [2,4,5,6,8];
for j=1:5
    numVant = vants(j)

    % Inicializar a matriz que armazenará todos os resultados
    todosResultados = [];
    
    % Loop sobre os arquivos .mat
    for i = 1:10
        nomeArquivo = sprintf('Experimento_%d-%dVANTS.mat', i,numVant);
        
        % Carregar o arquivo .mat
        dados = load(nomeArquivo);
        
        % Acessar a variável resultado
        resultadoAtual = dados.resultados(:,2)';
        
        % Concatenar o resultado na matriz geral
        todosResultados = [todosResultados; resultadoAtual];
    end
    
    % Agora, todosResultados contém os resultados de todos os arquivos .mat
    disp('Matriz combinada:');
    % disp(todosResultados);
    
    hold on
    subplot(2,3,num_fig)
    titulo = ["Experimentos com "+num2str(numVant)+" VANTs"];
    criarGraficoComIntervaloDeConfianca(todosResultados,titulo)
    num_fig = num_fig +1;

    % novo
%     mediaCustos = mean(todosResultados);
%     it = 1:1:10000;    
%     mediaCustos = [it; mediaCustos]';
end


function criarGraficoComIntervaloDeConfianca(todosResultados,titulo)
    
    iteracoes = [1:1:10000];

    % Calcular a média e o desvio padrão dos custos
    mediaCustos = mean(todosResultados);
    desvioPadraoCustos = std(todosResultados);

    % Calcular o intervalo de confiança de 10%
    intervaloConfianca = 0.1 * desvioPadraoCustos;

    % Criar um gráfico de linhas com intervalo de confiança
%     figure;
    plot(iteracoes, mediaCustos, 'LineWidth', 1,'Color','b','LineStyle','-');
    hold on;

    % Intervalo de confiança
    cima = mediaCustos + intervaloConfianca;
    baixo = mediaCustos - intervaloConfianca;

%     opts={'EdgeColor', 'g','FaceColor', [1 0.5 0.5]};
    opts={'EdgeColor', 'none','FaceColor', [1 0.5 0.5], 'MarkerEdgeColor','w','LineWidth', 0.2};

    % Adicionar a área sombreada entre as duas curvas
    fill_between(iteracoes, cima, baixo,[],opts{:});

    plot(iteracoes, cima,'Color','w');
    plot(iteracoes, baixo ,'Color','w');

%     hold off;

    % Configurações do gráfico    
    title(titulo);
    xlabel('Iterações');
    ylabel('Custo Médio');
    legend('Intervalo de confiança de 10%','Média');

    % Adicione outras configurações conforme necessário
end

