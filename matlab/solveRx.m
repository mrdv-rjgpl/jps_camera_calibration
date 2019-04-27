function Rx=solveRx( alphas, betas )

% alphas: A 3xN matrix representing the skew symmetric matrices. That
%         is, alphas=[α1,...,αN]
% betas: A 3xN matrix representing the skew symmetric matrices. That
%        is, betas=[β1,...,βN]
% return: The least squares solution to the matrix Rx
M = zeros(3, 3);

for i=1:size(alphas, 2)
	M(:, :) = M(:, :) + (betas(:, i) * alphas(:, i)');
end

[Mu, Md] = eig(M' * M);
Rx = Mu * sqrtm(inv(Md)) * Mu' * M';
