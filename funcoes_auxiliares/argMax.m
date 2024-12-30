function [s,s_index] = argMax(trs1,trs2,nrs,r,C,alpha,beta,p0)
    
    p = rand(); % valor aleatório

    feromonios = (trs1.^alpha).*(trs2.^(alpha*2)).*(nrs.^beta); % matriz de feromônios
    feromonios_rs = feromonios(r,C); % feromônios do caminho rs
    Prs = feromonios_rs./sum(feromonios_rs); % probabilidades dos feromônios dos caminhos rs serem escolhidos

    [~,s_index] = max(feromonios_rs); % verificando a tarefa com maior quantidade de feromônio    

    % condição de escolha
    if p<p0
        s = C(s_index); % tarefa com maior feromônio
    else
        % tarefa aleatória
        s_index = randi(length(C));
        s = C(s_index);
    end
      
    
end