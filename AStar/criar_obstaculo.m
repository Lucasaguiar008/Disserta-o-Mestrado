function obstaculo = criar_obstaculo(EixoY,EixoXi,EixoXf,passo)
    a= [EixoXi:passo:EixoXf]';
    tamanho = size(a,1);
    b = repmat(EixoY, 1, tamanho)';
    obstaculo = [a,b];
end