

function s_saturou = s_saturou(historico_s,qtd)
    
    s_saturou = false;
    
    % verificando as qtd últimas tarefas selecionadas  
    if (size(historico_s,2)>qtd)
        aux = historico_s(end-qtd:end);                
        if (aux == sort(aux))==true	                 
            s_saturou = true; 
        end
    end

end

