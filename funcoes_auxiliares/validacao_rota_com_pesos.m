

function validacao_rota = validacao_rota_com_pesos(formiga_i,P,cjk,xijk,distMax_voo,Pesos,capacidadeMaxima)
    i =formiga_i;    
    P{i} = [P{i} length(cjk)]; % Adicionando o retorno ao depósito (galpão)

    for j=1:length(P{i})-1
        r = P{i}(j);
        s = P{i}(j+1);
        xijk{i}(r,s) = 1;
    end

    Cp = custoParcial(xijk{i},cjk);  % Custo parcial de cada formiga  
    if (Cp<=distMax_voo(formiga_i) && sum(Pesos(P{i}))<=capacidadeMaxima )
        validacao_rota = true;
    else
        validacao_rota = false;
    end

end