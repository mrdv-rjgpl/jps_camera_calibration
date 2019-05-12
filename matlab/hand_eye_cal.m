rosshutdown;
rosinit;
tftree = rostf;
pause(1);
e_bh=[];
e_sc=[];

fprintf('Press any key to obtain the first transformation once the marker frame is visible. DO NOT CLICK THE MOUSE!\n');
k = waitforbuttonpress
num_frames_acq = 0;

while(k)
	fprintf('Getting transformation from camera_link to marker...\n');
	camera2marker = getTransform(tftree, 'camera_link', 'marker');
	fprintf('Getting transformation from base_link to ee_link...\n');
	base2ee = getTransform(tftree, 'base_link', 'ee_link');

	fprintf('Getting transformation components...\n');
	translation = camera2marker.Transform.Translation;
	rotation = camera2marker.Transform.Rotation;
	t_cam = [translation.X, translation.Y, translation.Z];
	r_cam = [rotation.X, rotation.Y, rotation.Z, rotation.W];

	fprintf('Computing e_sc...\n');
	e_sc=[e_sc;[t_cam r_cam]];

	translation = base2ee.Transform.Translation;
	rotation = base2ee.Transform.Rotation;
	t_fk =  [translation.X, translation.Y, translation.Z];
	r_fk = [rotation.X, rotation.Y, rotation.Z, rotation.W];

	fprintf('Computing e_bh...\n');
	e_bh=[e_bh;[t_fk r_fk]];
	num_frames_acq = num_frames_acq + 1
	fprintf('Obtained transformation. Press any key to obtain the next transformation. Click the mouse to begin the hand-eye calibration computations.\n');
	k = waitforbuttonpress
end

fprintf('All required transformations acquired. Computing hand-eye calibration...\n');
close all;
rosshutdown;
n=size(e_bh,1);
bin=nchoosek([1:n],n-1);

% for i=1:n-1
%     X=axxb(e_bh(bin(i,:),:), e_sc(bin(i,:),:));
%     X_translation = X(1:3,4);
%     X_rot = X(1:3,1:3);
%     X_temp = rotm2quat(X_rot);
%     %X_quat = [X_temp(4), X_temp(1:3)];
%     X_quat=[X_temp(2:4), X_temp(1)];
%     t(i,:)=[X_translation' X_quat];
% end
%

X=axxb(e_bh, e_sc);
X_translation = X(1:3,4);
X_rot = X(1:3,1:3);
X_temp = rotm2quat(X_rot);
X_quat = [X_temp(4), X_temp(1:3)];
X_quat=[X_temp(2:4), X_temp(1)];
X_final = [X_translation' X_quat];
fileID = fopen('../dat/camera_pose.txt', 'w');

for i = 1 : 7
	fprintf(fileID, '%f', X_final(i));

	if (i ~= 7)
		fprintf(fileID, ' ');
	else
		fprintf(fileID, '\n');
	end
end

save('../dat/hand_eye_cal.mat');

fclose(fileID);

function X=axxb( e_bh, e_sc )
	% e_bh: a Nx7 matrix that contain N forward kinematics measurements/home/anurag/matlabR2018b/code/Labs/lab 3
	%       obtained from tf_echo. The format of each row must be
	%       [t_x  t_y t_z  q_x  q_y  q_z  q_w]
	% e_sc: a Nx7 matrix that contain N AR tag measurements obtained from
	%       tf_echo. The format of each row must be
	%       [t_x  t_y  t_z  q_x  q_y q_z  q_w]
	% Return: the 4x4 homogeneous transformation of the hand-eye/home/anurag/matlabR2018b/code/Labs/lab 3
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

end
function Rx=solveRx( alphas, betas )

	% alphas: A 3xN matrix representing the skew symmetric matrices. That
	%         is, alphas=[α1,...,αN]/home/anurag/matlabR2018b/code/Labs/lab 3
	% betas: A 3xN matrix representing the skew symmetric matrices. That
	%        is, betas=[β1,...,βN]
	% return: The least squares solution to the matrix Rx
	M = zeros(3, 3);

	for i=1:size(alphas, 2)
		M(:, :) = M(:, :) + (betas(:, i) * alphas(:, i)');
	end

	[Mu, Md] = eig(M' * M);
	Rx = Mu * sqrtm(inv(Md)) * Mu' * M';


end


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

end
