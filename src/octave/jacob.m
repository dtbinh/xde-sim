%% INITIALIZATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;

pkg load symbolic;
pkg load nurbs;

nk = 5; % number non null intervals in knots vector
l = 2; % derivative order needed

spld = l + 2; % spline order
d = spld - 1; % here spline order is one less the value used in C++ code (eigen spline)
ncp = nk + spld - 1; % number of control points

%% SYMBOLIC VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = sym('X', [1 ncp*2]);
for i = 1:ncp*2
  assume(X(i), 'real');
end

Tp = sym('Tp', 'real', 'positive');

knots = knotsBases(spld, nk)*Tp;

index = linspace(spld-1, spld-1+nk-1, nk);

t = sym('t', 'real');

C = reshape(X, [2 ncp]);

o_pos = [sym('ox', 'real') sym('oy', 'real')];
t_pos = [sym('tx', 'real') sym('ty', 'real')];

%% CREATE B-spline from knots and C %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z = cell(1, size(index, 2));
dz = cell(1, size(index, 2));
ddz = cell(1, size(index, 2));
dddz = cell(1, size(index, 2));

g_pos = cell(1, size(index, 2));
g_vel = cell(1, size(index, 2));
g_absvel = cell(1, size(index, 2));
g_acc = cell(1, size(index, 2));
g_absacc = cell(1, size(index, 2));
g_dobst = cell(1, size(index, 2));

f = cell(1, size(index, 2));

[dC,dknots] = bspderivative(d, C, knots);
[ddC,ddknots] = bspderivative(d-1, dC, dknots);
[dddC,dddknots] = bspderivative(d-2, ddC, ddknots);

for i = 1:size(index, 2)

  z{i} = bspevaluation(index(i), d, C, knots, t);
  dz{i} = bspevaluation(index(i)-1, d-1, dC, dknots, t);
  ddz{i} = bspevaluation(index(i)-2, d-2, ddC, ddknots, t);
  dddz{i} = bspevaluation(index(i)-3, d-3, dddC, dddknots, t);

  f{i} =  (z{i}(1)-t_pos(1))^2 + (z{i}(2)-t_pos(2))^2;

%% CREATE CONSTRAINTS FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  g_pos{i} = [z{i}(1) z{i}(2) atan2(dz{i}(2), dz{i}(1))];

  g_vel{i} = [norm(dz{i}) (dz{i}(1)*ddz{i}(2) - dz{i}(2)*ddz{i}(1))/(dz{i}(1)^2 + dz{i}(2)^2)];

  g_absvel{i} = [norm(dz{i}) abs((dz{i}(1)*ddz{i}(2) - dz{i}(2)*ddz{i}(1))/(dz{i}(1)^2 + dz{i}(2)^2))];

  dv = (dz{i}(1)*ddz{i}(1) + dz{i}(2)*ddz{i}(2))/norm(dz{i});
  domega = ((ddz{i}(1)*ddz{i}(2)+dddz{i}(2)*dz{i}(1)-(ddz{i}(2)*ddz{i}(1)+dddz{i}(1)*dz{i}(2)))*norm(dz{i})^2 - 2*(dz{i}(1)*ddz{i}(2)-dz{i}(2)*ddz{i}(1))*norm(dz{i})*dv)/norm(dz{i})^4;

  g_acc{i} = [dv domega];

  g_absacc{i} = [abs(dv) abs(domega)];

  g_dobst{i} = -1.0*norm(transpose(z{i})-o_pos); % from the inequation form constraints(x) <= 0
  
end
%% COMPUTE SYMBOLIC JACOBIANS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic();
for i = 1:size(index, 2)

  disp('f...');
  J = jacobian(f{i}, X);
  save(["f" num2str(i) ".mat"], "J");
  ccJ = "J << ";
  for j = 1:size(J, 1)
    for w = 1:size(J, 2)
      if isequaln(true, J(j,w) == 0)
        ccJ = [ccJ "0.0"];
      else
        ccJ = [ccJ ccode(J(j, w))];
      end
      if j != size(J, 1) || w != size(J, 2)
        ccJ = [ccJ ", "];
      end
    end
    if j != size(J, 1)
      ccJ = [ccJ "\n"];
    end
  end
  ccJ = [ccJ ";"];
  filename = ["f" num2str(i) ".txt"];
  fid = fopen (filename, "w");
  fputs (fid, ccJ);
  fclose (fid);
  disp('done');


%  disp('g_pos...');
%  J = jacobian(g_pos{i}, X);
%  save(["q" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["qJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
%
%  disp('g_vel...');
%  J = jacobian(g_vel{i}, X);
%  save(["dq" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["dqJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
%
%  disp('g_absvel...');
%  J = jacobian(g_absvel{i}, X);
%  save(["absdq" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["absdqJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
%
%  disp('g_acc...');
%  J = jacobian(g_acc{i}, X);
%  save(["ddq" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["ddqJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
%
%  disp('g_absacc...');
%  J = jacobian(g_absacc{i}, X);
%  save(["absddq" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["absddqJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
%
%  disp('g_dobst...');
%  J = jacobian([g_dobst{i}], X);
%  save(["dobst" num2str(i) ".mat"], "J");
%  ccJ = "J << ";
%  for j = 1:size(J, 1)
%    for w = 1:size(J, 2)
%      if isequaln(true, J(j,w) == 0)
%        ccJ = [ccJ "0.0"];
%      else
%        ccJ = [ccJ ccode(J(j, w))];
%      end
%      if j != size(J, 1) || w != size(J, 2)
%        ccJ = [ccJ ", "];
%      end
%    end
%    if j != size(J, 1)
%      ccJ = [ccJ "\n"];
%    end
%  end
%  ccJ = [ccJ ";"];
%  filename = ["distRObsJac" num2str(i) ".txt"];
%  fid = fopen (filename, "w");
%  fputs (fid, ccJ);
%  fclose (fid);
%  disp('done');
end
disp(toc());
save("allJac.mat")