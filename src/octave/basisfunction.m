function N = basisfunction (i, u, p, U)

% Basis Function. 
%
% INPUT:
%
%   i - knot span  ( from FindSpan() )
%   u - parametric point
%   p - spline degree
%   U - knot sequence
%
% OUTPUT:
%
%   N - Basis functions vector[p+1]
%
% Algorithm A2.2 from 'The NURBS BOOK' pg70.

  left = sym('left', [1, p+1]);
  right = sym('right', [1, p+1]);
  
  N = sym('N', [1 p+1]);
  N(:,:) = 0.0;
  
  N(1,1) = 1.0;
  
  for j = 1:p
    left(j) = u - U(i+1-j + 1);
    right(j) = U(i+j + 1) - u;
    
    syms saved;
    saved = 0.0;
    
    for r = 0:j-1
      temp = N(r+1) / (right(r+1) + left(j-r));
      N(r+1) = saved + right(r+1) * temp;
      saved = left(j-r) * temp;
    endfor
    N(1,j+1) = saved;
  endfor

