function [dC dknots] = bspderivative(d, C, knots)

  mc = size(C, 1);
  nc = size(C, 2);
  nk = size(knots, 2);
  
  dC = sym('dC', [mc nc-1]);
  dknots = sym('dknots', [1 nk-2]);
  
  for i = 1:nc-1
    tmp = d/(knots(i+d+1) - knots(i+1));
    for j = 1:mc
      dC(j,i) = tmp*(C(j,i+1) - C(j,i));
    endfor
  endfor
  
  for i = 2:nk-1
    dknots(i-1) = knots(i);
  endfor
  
endfunction