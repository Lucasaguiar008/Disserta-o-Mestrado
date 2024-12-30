%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Disciplina: Estudos Especiais - Mestrado UFC 31/10/2023

clc; clear; close all;

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("Resultados\")

%%

% vants = [2,4,5,6,8];

% Inicializar a matriz que armazenará todos os resultados
todosResultados = [];

% Loop sobre os arquivos .mat
for i = 1:10
    nomeArquivo = sprintf('Experimento_%d-6VANTS.mat', i);
    
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


%%
criarGraficoComIntervaloDeConfianca(todosResultados)


function criarGraficoComIntervaloDeConfianca(todosResultados)
    
    iteracoes = [1:1:10000];

    % Calcular a média e o desvio padrão dos custos
    mediaCustos = mean(todosResultados);
    desvioPadraoCustos = std(todosResultados);

    % Calcular o intervalo de confiança de 10%
    intervaloConfianca = 0.1 * desvioPadraoCustos;

    % Criar um gráfico de linhas com intervalo de confiança
    figure;
    plot(iteracoes, mediaCustos, 'LineWidth', 1,'Color','b');
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
    title('Gráfico de Linhas com Intervalo de Confiança de 10%');
    xlabel('Iterações do Experimento');
    ylabel('Custo Médio');
    legend('Média', 'Intervalo de Confiança');

    % Adicione outras configurações conforme necessário
end

