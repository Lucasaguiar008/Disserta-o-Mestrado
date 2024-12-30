function [trs1,trs2] = att_Feromonio(trs1o,trs2o,trs1,trs2,rho,r,s)    
    trs1(r,s) = (1-rho).*trs1(r,s) + rho.*trs1o(r,s);
    trs2(r,s) = (1-rho).*trs2(r,s) + rho.*trs2o(r,s);
end