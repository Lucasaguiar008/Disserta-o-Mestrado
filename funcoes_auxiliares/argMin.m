function [s,s_index] = argMin(cjk,r,C)
    [~,s_index] = min(cjk(r,C));
    s = C(s_index);
end