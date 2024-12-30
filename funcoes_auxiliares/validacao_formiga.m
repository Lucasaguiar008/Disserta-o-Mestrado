

function [validacao_formiga, P, C] = validacao_formiga(formiga_i,P,cjk,xijk,distMax_voo,C,s_saturou)    
    i =formiga_i;     
    rotas_validas = [];

    for j=1:length(P{i})-1
        r = P{i}(j);
        s = P{i}(j+1);
        xijk{i}(r,s) = 1;
    end
    Cp = custoParcial(xijk{i},cjk);  % Custo parcial dea rota
                   
    dist_rotas_possiveis = cjk(s,C); %rotas possíveis        
    dist_retornos_possiveis = cjk(end,C);
    dist_rotas = Cp + dist_rotas_possiveis + dist_retornos_possiveis;

    rotas_validas = dist_rotas(dist_rotas<distMax_voo(i));    

    % se houver rotas validas a formiga está validada a continuar no algoritmo
    if isempty(rotas_validas)
        validacao_formiga = false;
    else        
        validacao_formiga = true;

        if s_saturou
            s_novo = C(dist_rotas==min(rotas_validas)); % indica a melhor rota, as vezes tem mais de 1 elemento

            if length(s_novo)>1                
                indices = ismember(C, s_novo); %verifica os indices das rotas candidatas
                [~,aux] = min(dist_rotas_possiveis(indices));% pega a rota com menor dist
                s_novo = s_novo(aux); % setando a rota
            end
            P{formiga_i} = [P{formiga_i} s_novo]; % Atribuindo a tarefa s_novo para a formiga î
            C(C==s_novo) = []; % Remove s_novo de C;
        end
    end
            

end