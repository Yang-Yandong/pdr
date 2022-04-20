function DCM_nb = eulr2dcm(eul_vect)
% EULR2DCM    Euler angle vector to direction cosine matrix conversion.欧拉角矢量到方向余弦矩阵转换
%{
%   INPUTS
%       eul_vect(1) = roll,  /radians 
%       eul_vect(2) = pitch, /radians 
%       eul_vect(3) = yaw, /radians 

%   OUTPUTS
%       DCMnb = 3x3 direction cosine matrix 
%       providing the transformation from the navigation frame to the body frame提供从导航框架到车身框架的转换

%   REFERENCE:  Titterton, D. and J. Weston, 
                STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY, 
                Peter Peregrinus Ltd. 
                on behalf of the Institution of Electrical Engineers, London, 1997. 
%}

%if nargin<1,error('insufficient number of input arguments'),end

phi = eul_vect(1); theta = eul_vect(2); psi = eul_vect(3);

cpsi = cos(psi); spsi = sin(psi);   % Yaw
cthe = cos(theta); sthe = sin(theta); % Pith
cphi = cos(phi); sphi = sin(phi); % Roll

C1 = [cpsi  spsi 0; 
      -spsi cpsi 0; 
       0     0   1];  % Z axis
C2 = [cthe  0  -sthe; 
        0   1     0 ; 
      sthe  0   cthe]; % y axis
C3 = [1   0    0;   
      0  cphi sphi;  
      0 -sphi cphi];  % x axis

DCM_nb = C3 * C2 * C1;


