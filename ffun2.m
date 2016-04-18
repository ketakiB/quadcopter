function [ nextstate ] = ffun2( stateinput )

nextstate = state + T_s*ffun(stateinput);

end

