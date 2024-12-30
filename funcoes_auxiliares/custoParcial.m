function custo = custoParcial(xijk,cjk)    
    custo = sum(sum(xijk.*cjk));
end