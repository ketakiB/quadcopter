function [ nextstate ] = ffun2( stateinput )
state = stateinput(1:12);
nextstate = state + 0.05*ffun(stateinput);

end

