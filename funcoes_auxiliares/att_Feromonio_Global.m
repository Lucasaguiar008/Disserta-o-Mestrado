
function [trs1,trs2] = att_Feromonio_Global(trs1o,trs2o,trs1,trs2,rho,f1,f2,Nv,P)

    trs1 = (1-rho).*trs1 + rho.*trs1o;
    trs2 = (1-rho).*trs2 + rho.*trs2o;

    for k=1:Nv
      % Reforçar o feronômio caminho encontrado
      for i = 1:(length(P{k}) - 1)
          citdade_i = P{k}(i);
          citdade_j = P{k}(i+1);
          trs1(citdade_i, citdade_j) = trs1(citdade_i, citdade_j) + 1 / f1;
          trs1(citdade_j, citdade_i) = trs1(citdade_i, citdade_j);  % Matriz simétrica

          trs2(citdade_i, citdade_j) = trs2(citdade_i, citdade_j) + 1 / (Nv*f2);
          trs2(citdade_j, citdade_i) = trs2(citdade_i, citdade_j);  % Matriz simétrica
      end
    end
end




%% Antigo (mudei para a versão que fiz no octave)

% function [trs1,trs2] = att_Feromonio_Global(trs1o,trs2o,trs1,trs2,rho,f1,f2)    
% 
%     nk=1;
%     deltaTrs1 = trs1o + 1/(nk*f1);
% 
%     nk=2;
%     deltaTrs2 = trs2o + 1/(nk*f2);
% 
%     trs1 = (1-rho).*trs1 + rho.*deltaTrs1;
%     trs2 = (1-rho).*trs2 + rho.*deltaTrs2;
% end