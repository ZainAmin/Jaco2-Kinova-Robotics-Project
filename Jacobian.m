% Function for making the Jacobian Matrix from the DH Table

function [J] = Jacobians(DH)

% Input DH Table
% Output Jacobian Matrix

n = size(DH, 1); % number of joints (number of rows of DH table) 
p0 = [0 0 0]';   % position of link 0 at the origin (P0 = all the coordinates x,y,z = 0)
z0 = [0 0 1]';   % position of z0 at link 0 (z0 = coordinates x,y =0,0 and z =1 because unit vector in the direction of z)
p = zeros(3,n);  % matrix constructed for the position vector of links [p1 p2 ...., pn, pn=pe, where pe indicates position of the end effector]


% loop for calcultaing homogeneous matrix, for n number of link

for i=1:n
    T_i = Homogeneous(DH(i,:));     % Calculating homogeneous matrix of the ith link
                                    % Input: i-th row DH table 
                                    % Output: Homogeneous matrix of the i-th row;
                                    % where i-th row indicates each link of the robot

    if (i==1)                       % if the link of the the robot is 1
        T(:, :, i) = T_i;           % inserting the homogeneous matrix of the link 1 into the n dimension homogeneous matrix
                                    % where nth dimension indicates number 
    end

    if (i>=2)                       % if the link of the robot is more than 1 (i>1)
        T(:, :, i) = T(:, :, i-1)*T_i; 
    end

    T0i = T(:, :, i);
    p(:, i) = T0i(1:3, 4);
end
    pe = p(:, n);

    J(:,1) = [cross(z0, pe-p0); z0];

    for i=2:n
        T0im1 = T(:,:,i-1);
        R0im1 = T0im1(1:3, 1:3);
        z0im1 = R0im1(:,3);
        p0im1 = p(:,i-1);

        J(:,i) = [cross(z0im1, (pe-p0im1)); z0im1];
        
    end
 end
