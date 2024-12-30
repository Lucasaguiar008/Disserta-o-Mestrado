%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Algoritmo ACO puro para parametrização
clc; clear; close all;

%% Leitor dos resultados

%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("resultado_parametrizacao\NG") %NG
addpath("resultado_parametrizacao\ALPHA") %ALPHA
addpath("resultado_parametrizacao\BETA") %BETA
addpath("resultado_parametrizacao\RHO") %rho
addpath("resultado_parametrizacao\p0") %p0
addpath("resultado_parametrizacao\q0") %q0
addpath("resultado_parametrizacao\q1") %q0

for simu=1:200                
        nome_arq = "resultado_parametrizacao/q0/simu_"+simu+"_q0_"+0.4+".mat";
        load(nome_arq);

        
        nome_arq_novo = "resultado_parametrizacao/q1/simu_"+simu+"_q1_"+0.05+".mat";
        save(nome_arq_novo)
end    