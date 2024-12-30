

function valor = pesosUnitarios(P,Nv)
    valor = 0;
    for i=1:Nv
        valor = valor + length(P{i});
    end
end