function [x,y] = Ler_mapa(nome_mapa, inicio)
    % A variável inicio indica a linha que começa o mapa
   
    %Lê o arquivo .tsp usando a função readtable, que lê a tabela de coordenadas no arquivo.
    % Observe que o argumento 'HeaderLines' é definido como 7 porque as seis primeiras linhas do arquivo .tsp 
    % contêm informações sobre o problema e não fazem parte da tabela de coordenadas. 
    % O argumento 'ReadVariableNames' é definido como false porque a tabela não contém nomes de variáveis.
            
    T = readtable(nome_mapa, 'FileType', 'text', 'HeaderLines', inicio, 'ReadVariableNames', false);
    
    % Retirando a última linha, pois não faz parte do grafo
    dim = (1:(size(T,1)-1));
    T = T(dim,:);
    
    %Extraindo as coordenadas dos nós da tabela em duas matrizes separadas, x e y.
    x = T.Var2;
    y = T.Var3;
        
    %plot(x,y,'square') % plontando os pontos.

end