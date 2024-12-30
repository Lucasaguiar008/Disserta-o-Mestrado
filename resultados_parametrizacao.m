%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Algoritmo ACO puro para parametrização
clc; clear; close all;

%% Leitor dos resultados

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("resultado_parametrizacao\NG") %NG
addpath("resultado_parametrizacao\ALPHA") %ALPHA
addpath("resultado_parametrizacao\BETA") % BETA
addpath("resultado_parametrizacao\RHO") % RHO
addpath("resultado_parametrizacao\p0") % p0
addpath("resultado_parametrizacao\q0") % q0



%% MAIN

vet_parametrizacao = [10,25,50,100]; %NG
% vet_parametrizacao = [0.5,1,2,4]; %alpha
% vet_parametrizacao = [0.5,1,2,4]; % beta
% vet_parametrizacao = [0.25,0.5, 0.75,1]; % rho
% vet_parametrizacao = [0.1,0.4,0.7, 0.9]; %p0
% vet_parametrizacao = [0.1, 0.4, 0.7, 0.9]; %q0
% vet_parametrizacao = [0.05, 0.1, 0.2, 0.3]; %q1

cores = ['b','k', 'g','r','m','c','y','k']; % Cores dos plots
legenda = [];

for i = 1:4    
    param = vet_parametrizacao(i);
    resultados_1 = zeros(100,1000);
    for simu=1:200
        nome_arq = "resultado_parametrizacao/NG/simu_"+simu+"_Ng_"+param+".mat"; %NG
        % nome_arq = "resultado_parametrizacao/ALPHA/simu_"+simu+"_alpha_"+param+".mat"; %alpha
        % nome_arq = "resultado_parametrizacao/BETA/simu_"+simu+"_beta_"+param+".mat"; %beta
        % nome_arq = "resultado_parametrizacao/RHO/simu_"+simu+"_rho_"+param+".mat"; % rho
        % nome_arq = "resultado_parametrizacao/p0/simu_"+simu+"_p0_"+param+".mat"; %p0
        % nome_arq = "resultado_parametrizacao/q0/simu_"+simu+"_q0_"+param+".mat"; %q0
        % nome_arq = "resultado_parametrizacao/q1/simu_"+simu+"_q1_"+param+".mat"; %q1

        load(nome_arq,'resultados');
        resultados_1(simu,:) = resultados(:,2)';
    end    
    eixo_x = resultados(:,1)';
    media = mean(resultados_1);
    maximo = max(resultados_1);
    minimo = min(resultados_1);
    desvPad = std(resultados_1);   
    
    s_param = "q1";
    val_param = param;
    legenda = [legenda, plot_esperanca(resultados_1,cores(i),s_param,val_param)];

    %% Melhor experimento
    if param==100
        plot(eixo_x,maximo,'Color','r')
        plot(eixo_x,minimo,'Color','g')
    end
end

% Adicionar títulos e legendas
title("Gráfico de Linha com Intervalo de Confiança de 95%");
xlabel('Interações');
ylabel('Valor Médio das Simulações');
legend(legenda);

%% Teste

function legenda= plot_esperanca(resultados,cor,s_param,val_param)
    % Supondo que "resultados" seja uma matriz de 100x1000
    % com 100 simulações e 1000 interações
    
    % Calcular a média ao longo das simulações para a linha central
    media_resultados = mean(resultados, 1);       

    % Calcular o intervalo de confiança (percentis 2.5% e 97.5%)
    intervalo_confianca = 95;
    esperanca = ((100-intervalo_confianca)/2);
    inferior = prctile(resultados, esperanca , 1);
    superior = prctile(resultados, intervalo_confianca, 1);

    % Suavização com janela de 5 pontos
    % media_resultados = movmean(mean(resultados, 1), 5);  % Suavização com janela de 5 pontos
    % inferior = movmean(prctile(resultados, 2.5, 1), 5);
    % superior = movmean(prctile(resultados, 97.5, 1), 5);

    
    % Plotar o gráfico com intervalo de confiança
    x = 1:size(resultados,2);  % Eixo X representa as interações
    
    % figure;
    fill([x, fliplr(x)], [superior, fliplr(inferior)], cor, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on;
    plot(x, media_resultados, cor, 'LineWidth', 1.5);
    legenda = ["Interv. de confiança: "+s_param+" = "+val_param, "Valor médio: "+s_param+" = "+val_param];
    hold on;
    
end
