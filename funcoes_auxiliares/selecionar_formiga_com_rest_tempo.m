function formiga = selecionar_formiga_com_rest_tempo(P,cjk,q0,q1,formigas_disponiveis)
    q = rand(); % número aleatório entre 0 e 1
    Nv = length(P); % quantidade de formigas
        
    Custo = zeros(1,Nv);
    for i=1:Nv        
        for j=1:length(P{i})-1                        
            r = P{i}(j);
            s = P{i}(j+1);            
            Custo(i) = Custo(i) + cjk(r,s);
        end
    end
    
    Custo_formigas_disponiveis = Custo(formigas_disponiveis);

    if q<q0
        [~,indice] = min(Custo_formigas_disponiveis);
        formiga = formigas_disponiveis(indice);
    elseif q>q1
        [~,indice] = max(Custo_formigas_disponiveis);
        formiga = formigas_disponiveis(indice);
    else
        formiga = formigas_disponiveis(randi(Nv));
    end
end