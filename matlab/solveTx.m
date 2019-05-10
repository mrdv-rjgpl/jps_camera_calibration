function tx=solveTx( RA, tA, RB, tB, RX )
% RA: a 3x3xN matrix with all the rotations matrices R_(A_i )
% tA: a 3xN matrix with all the translation vectors t_(A_i )
% RB: a 3x3xN matrix with all the rotations matrices R_(B_i )
% tB: a 3xN matrix with all the translation vectors t_(B_i )
% RX: the 3x3 rotation matrix Rx
% return: the 3x1 translation vector tx

ra_reshaped = zeros(size(RA, 1) * size(RA, 3), size(RA, 2));
ta_reshaped = reshape(tA, 3*size(tA, 2), 1);
tb_reshaped = reshape(tB, 3*size(tB, 2), 1);

for i=1:size(RA, 3)
	ra_reshaped(((3 * i) - 2):(3 * i), :) = RA(:, :, i) - eye(3);
	tb_reshaped(((3 * i) - 2):(3 * i), :) = RX * tb_reshaped(((3 * i) - 2):(3 * i), :);
end

tx = inv(ra_reshaped' * ra_reshaped) * ra_reshaped' * (tb_reshaped - ta_reshaped);
