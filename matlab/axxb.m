function X=axxb( e_bh, e_sc )
% e_bh: a Nx7 matrix that contain N forward kinematics measurements
%       obtained from tf_echo. The format of each row must be
%       [t_x  t_y t_z  q_x  q_y  q_z  q_w]
% e_sc: a Nx7 matrix that contain N AR tag measurements obtained from
%       tf_echo. The format of each row must be
%       [t_x  t_y  t_z  q_x  q_y q_z  q_w]
% Return: the 4x4 homogeneous transformation of the hand-eye
%         calibration
alphas = zeros(3, size(e_bh, 1) - 1);
betas = zeros(3, size(e_sc, 1) - 1);
tA = zeros(3, size(e_bh, 1) - 1);
tB = zeros(3, size(e_bh, 1) - 1);
RA = zeros(3, 3, size(e_bh, 1) - 1);
RB = zeros(3, 3, size(e_sc, 1) - 1);

% Find the relative motions of the robot and sensor frames, from their given instantaneous positions.
e_bh_1_rot_inv = inv(quat2rotm([e_bh(1, 7), e_bh(1, 4:6)]));
e_sc_1_rot = quat2rotm([e_sc(1, 7), e_sc(1, 4:6)]);

for i=2:size(e_bh, 1)
	RA(:, :, i - 1) = e_bh_1_rot_inv * quat2rotm([e_bh(i, 7), e_bh(i, 4:6)]);
	rot_m_log = logm(RA(:, :, i - 1));
	alphas(:, i - 1) = [rot_m_log(3, 2), rot_m_log(1, 3), rot_m_log(2, 1)]';
	tA(:, i - 1) = e_bh_1_rot_inv * (e_bh(i, 1:3) - e_bh(1, 1:3))';

	RB(:, :, i - 1) = e_sc_1_rot * inv(quat2rotm([e_sc(i, 7), e_sc(i, 4:6)]));
	rot_m_log = logm(RB(:, :, i - 1));
	betas(:, i - 1) = [rot_m_log(3, 2), rot_m_log(1, 3), rot_m_log(2, 1)]';
	tB(:, i - 1) = e_sc(1, 1:3)' - (RB(:, :, i - 1) * e_sc(i, 1:3)');
end

Rx = solveRx(alphas, betas);
tx = solveTx(RA, tA, RB, tB, Rx);
X = [Rx, tx; 0, 0, 0, 1];
