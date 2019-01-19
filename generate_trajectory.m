
function [p_eq_val, v] = generate_trajectory( n , r, constraints, inequality_constraint, timestep)

syms t t_i t_f;

coefficient_array = fliplr(sym('a%d', [1 n+1]));
% disp(coefficient_array);

p = sym('p', [r+1 1]);
p(1,1) = poly2sym(coefficient_array,t);

for i=1:r
    p(i+1,1) = diff(p(i,1));
end
% disp(x);

% cost = int(p(r+1,1)^2, [0, 1]);
cost = int(p(r+1,1)^2, [t_i,t_f]);
% disp(cost);

H =  hessian(cost, fliplr(coefficient_array));
% disp(H);

H_val = subs(H, [t_i, t_f], [constraints(1,2), constraints(2,2)]);
% disp(H_val);

A_equations = sym('a%d', [size(constraints, 1), 1]);
% disp(A_equations);

for i=1:size(constraints,1)
%     disp(p(constraints(i,3)+1));
    A_equations(i) = subs(p(constraints(i,3)+1), t, constraints(i,2)) == constraints(i,1);
end
% disp(A_equations);

% disp(double(Aeq));
% disp(double(beq));
[Aeq,beq] = equationsToMatrix(A_equations,fliplr(coefficient_array));

A = [];
b = [];
if isempty(inequality_constraint)==0
    A_inequality_equations = sym('ae%d', [size(inequality_constraint, 1), 1]);
  
    for i=1:size(inequality_constraint,1)
       A_inequality_equations(i) = subs(p(inequality_constraint(i,3)+1), t, inequality_constraint(i,2)) == inequality_constraint(i,1);
    end

   [A,b] = equationsToMatrix(A_inequality_equations,fliplr(coefficient_array));  
end
% disp(fliplr(coefficient_array)');
coefficient_array_val = quadprog(double(H_val), [], double(A), double(b), double(Aeq), double(beq));
% disp(coefficient_array_val);

% disp(fliplr(coefficient_array_val)');

p_eq = sym('p_eq', [r+1 1]);

for i=1:(r+1)
    p_eq(i,1) = subs(p(i,1), fliplr(coefficient_array), fliplr(coefficient_array_val)' );
end

N = constraints(2,2)/timestep;
N = int16(N);
p_eq_val = zeros(r+1, N);

time = 1;
for t_val=constraints(1,2)+timestep:timestep:constraints(2,2)
    for i=1:(r+1)
        p_eq_val(i,time) = eval(subs(p_eq(i,1), t, t_val ));
    end
    time = time+1;
end

v = p_eq_val(2,end);
end