
%% DADOS DO AUTOR
% Aluno: Antônio Lucas Sousa Aguiar
% Algoritmo ACO puro para parametrização

clc; clear; close all;


%% PASTA DA FUNÇÕES QUE SERÃO USADAS
addpath("funcoes_auxiliares\")
% nome_arq = "resultado_parametrizacao/NG/simu_"+171+"_Ng_"+100+".mat"; %NG
% nome_arq = "resultado_parametrizacao/ALPHA/simu_"+145+"_alpha_"+0.5+".mat"; %alpha
% nome_arq = "resultado_parametrizacao/BETA/simu_"+77+"_beta_"+4+".mat"; %beta
% nome_arq = "resultado_parametrizacao/RHO/simu_"+187+"_rho_"+0.25+".mat"; % rho
% nome_arq = "resultado_parametrizacao/p0/simu_"+187+"_p0_"+0.9+".mat"; %p0
% nome_arq = "resultado_parametrizacao/q0/simu_"+78+"_q0_"+0.4+".mat"; %q0
nome_arq = "resultado_parametrizacao/q1/simu_"+181+"_q1_"+0.3+".mat"; %q1
load(nome_arq)


%% PLOTANDO RESULTADOS

% Plotando o mapa
hold off

% Galpão inicial e final
figure(1);
galpaoIncicial = [1,1];
plot(galpaoIncicial(1),galpaoIncicial(2),'b--o','MarkerSize' ,10);
text(galpaoIncicial(1)+0.2,galpaoIncicial(2)-0.2,'Galpao')
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
title("Rotas Geradas")
xlabel("Posição no eixo X")
ylabel("Posição no eixo Y")
numfig = 1;
plotando_caminhos_aco_puro(P_best,Nv,x,y,numfig)

%%
disp('Custos: ')
Cp_best
disp('f1: ')
sum(Cp_best)
disp('f2: ')
max(Cp_best)

P_best