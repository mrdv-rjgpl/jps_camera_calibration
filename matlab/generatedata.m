% Generate synthetic data to test hand-eye calibration.
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ]
% were tx, ty and tz denote a translation and qx, qy, qz, qw a
% quaternion.
% X is a randomly generated hand-eye transformation
function [e_bh, e_sc, X] = generatedata(N)
% Assume we know the transformation between the base and the
% checkerboard
E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ];
% Create a random X for generating the data
X = randSE3();
e_bh = [];
e_sc = [];

%fh = fopen('inputs.txt', 'w');
%fprintf(fh, '# ee_link\n');

for i=1:N
	% Now that you have X and E_bc, generate a random E_bh, you can use rotm2quat to convert the rotation matrix to a
	% quaternion (be careful because it outputs in the format [qw, qx, qy, qz]) and append the transformation to e_bh
	E_bh = randSE3();
	e_bh = [e_bh; tfMat2Row(E_bh)];
	% Now that you have X, E_bc and E_bh, find E_sc and append the transformation to e_sc
	E_sc = inv(E_bh * X) * E_bc;
	e_sc = [e_sc; tfMat2Row(E_sc)];
end

%for i=1:N
%	fprintf(fh, '%.15f %.15f %.15f %.15f %.15f %.15f %.15f\n', e_bh(i, 1), e_bh(i, 2), e_bh(i, 3), e_bh(i, 4), e_bh(i, 5), e_bh(i, 6), e_bh(i, 7));
%end

%fprintf(fh, '# marker\n');

%for i=1:N
%	fprintf(fh, '%.15f %.15f %.15f %.15f %.15f %.15f %.15f\n', e_sc(i, 1), e_sc(i, 2), e_sc(i, 3), e_sc(i, 4), e_sc(i, 5), e_sc(i, 6), e_sc(i, 7));
%end

%fclose(fh);

% Generate a random SE3 transformation
function Rt = randSE3()
% Generate a random rotation matrix
w = (rand(3, 1) - 0.5) * 4.0 * pi / sqrt(3); % Limit to [-2pi, 2pi]
R = expm([0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0]); % Exponential of skew symmetric matrix.
% Generate a random translation
t = (rand(3, 1) -0.5) * 0.4; % Limit to 0.2 m in each direction
Rt = [ R t; 0 0 0 1];

function tf_row = tfMat2Row(tf_mat)
q = rotm2quat(tf_mat(1:3, 1:3));
tf_row = [tf_mat(1:3, 4)' q(2:4) q(1)];
