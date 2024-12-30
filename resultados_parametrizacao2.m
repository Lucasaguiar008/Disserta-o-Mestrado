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
 
    legenda = [legenda,"q1 = "+ param];    
    plot(eixo_x,media,'Color',cores(i))  

    % plot_esperanca(resultados_1,cores(i))
    hold on

    media = media(end);
    maximo = maximo(end);
    minimo = minimo(end);
    desvPad = desvPad(end);

    disp("Resultado_Param: "+param);
    disp("Média: "+media);
    disp("Minimo: "+minimo);
    disp("Maximo: "+maximo);
    disp("Desvio_Padrao: "+desvPad);
    disp(' ');
    disp(' ');
    
    %% Melhor experimento
    if param==100
        result_melhor = resultados_1(:,end);
        [M,I] = min(result_melhor)
    end
    %%

    
end

legend(legenda);
xlabel('Iterações')
ylabel('Custo em metros')
title('Resultado parametrização')

%% Teste

function plot_esperanca(resultados,cor)
    % Supondo que "resultados" seja uma matriz de 100x1000
    % com 100 simulações e 1000 interações
    
    % Calcular a média ao longo das simulações para a linha central
    media_resultados = mean(resultados, 1);       

    % Calcular o intervalo de confiança (percentis 2.5% e 97.5%)
    intervalo_confianca = 90;
    esperanca = ((100-intervalo_confianca)/2);
    inferior = prctile(resultados, esperanca , 1);
    superior = prctile(resultados, intervalo_confianca, 1);

    % Suavização com janela de 5 pontos
    media_resultados = movmean(mean(resultados, 1), 5);  % Suavização com janela de 5 pontos
    % inferior = movmean(prctile(resultados, 2.5, 1), 5);
    % superior = movmean(prctile(resultados, 97.5, 1), 5);

    
    % Plotar o gráfico com intervalo de confiança
    x = 1:1000;  % Eixo X representa as interações
    
    % figure;
    fill([x, fliplr(x)], [superior, fliplr(inferior)], cor, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on;
    plot(x, media_resultados, cor, 'LineWidth', 1.5);
    
    % Adicionar títulos e legendas
    title("Gráfico de Linha com Intervalo de Esperança de "+esperanca*2+"%");
    xlabel('Iterações');
    ylabel('Valor Médio das Simulações');
    legend("Esperança de "+esperanca*2+"%", 'Média das Simulações');
    hold on;
end
