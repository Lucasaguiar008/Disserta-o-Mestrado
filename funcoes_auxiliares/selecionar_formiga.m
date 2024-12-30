function formiga = selecionar_formiga(P,cjk,q0,q1)
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

    if q<q0
        [~,formiga] = min(Custo);
    elseif q>q1
        [~,formiga] = max(Custo);
    else
        formiga = randi(Nv);
    end
end