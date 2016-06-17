function z = bspevaluation(i, d, c, k, u)
  
  nu = size(u, 2);
  mc = size(c, 1);
  nc = size(c, 2);
  
  p = sym('p', [mc nu]);
  p(:,:) = 0.0;

  %N = sym('N', [1 d+1]);
  %N(:,:) = 0.0;
  
  for col = 1:nu
    s = i;
    N = basisfunction(s, u(col), d, k);
    #size(N)
    tmp1 = s - d;
    for row = 1:mc
      syms tmp2;
      tmp2 = 0.0;
      for j = 1:d+1
        #tmp1+j
        #size(c)
        tp1 = c(row, tmp1+j);
        tp2 = N(j);
        tmp2 = tmp2 + tp2*tp1;
      endfor
      p(row, col) = tmp2;
    endfor
  endfor
  
  z = p;
  
endfunction