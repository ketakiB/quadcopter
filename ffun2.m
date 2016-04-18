function [ nextstate ] = ffun2( stateinput )
state = stateinput(1:12);
nextstate = state + T_s*ffun(stateinput);

end

