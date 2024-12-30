

function valor = PesosColetados(P,Nv,Pesos)
    valor = 0;
    for i=1:Nv        
        valor = valor +  sum(Pesos(P{1}));        
    end
end