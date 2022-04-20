function qua_vec = dcm2qua(DCMbn)
%DCM2QUA       Direction cosine matrix to quaternion（四元数） conversion.
%       方向余弦矩阵到四元数的转换
%	qua_vec = (DCMbn)
%
%   INPUTS
%       DCMbn = 3x3 direction cosine matrix providing the
%             transformation from the body frame
%             to the navigation frame
%dcm2qua
%   OUTPUTS
%       qua_vec = 4 element quaternion vector
%               = [a b c d]
%       where: a = cos(MU/2)
%              b = (MUx/MU)*sin(MU/2)
%              c = (MUy/MU)*sin(MU/2)
%              d = (MUz/MU)*sin(MU/2)
%       where: MUx, MUy, MUz are the components of the angle vector
%              MU is the magnitude of the angle vector
%
%   NOTE same with AIMS in TRANSFC.CPP
%
%	Copyright (c) 2006 by Jack Wang
%	All Rights Reserved.
%
if nargin<1,error('insufficient number of input arguments'),end

s(5) = DCMbn(1,1) + DCMbn(2,2) + DCMbn(3,3);
s(1) = 1.0 + s(5);
s(2) = 1.0 + 2.0*DCMbn(1,1) - s(5);
s(3) = 1.0 + 2.0*DCMbn(2,2) - s(5);
s(4) = 1.0 + 2.0*DCMbn(3,3) - s(5);

% get max value index */
[cmax,imax]= max(s);
	switch (imax)
       case 1
			qua_vec(1) = 0.5*sqrt(s(1));
			qua_vec(2) = 0.25*(DCMbn(3,2) - DCMbn(2,3))/qua_vec(1);
			qua_vec(3) = 0.25*(DCMbn(1,3) - DCMbn(3,1))/qua_vec(1);
			qua_vec(4) = 0.25*(DCMbn(2,1) - DCMbn(1,2))/qua_vec(1);
       case 2
			qua_vec(2) = 0.5*sqrt(s(2));
			qua_vec(3) = 0.25*(DCMbn(2,1) + DCMbn(1,2))/qua_vec(2);
			qua_vec(4) = 0.25*(DCMbn(1,3) + DCMbn(3,1))/qua_vec(2);
			qua_vec(1) = 0.25*(DCMbn(3,2) - DCMbn(2,3))/qua_vec(2);
       case 3
			qua_vec(3) = 0.5*sqrt(s(3));
			qua_vec(4) = 0.25*(DCMbn(3,2) + DCMbn(2,3))/qua_vec(3);
			qua_vec(1) = 0.25*(DCMbn(1,3) - DCMbn(3,1))/qua_vec(3);
			qua_vec(2) = 0.25*(DCMbn(2,1) + DCMbn(1,2))/qua_vec(3);
       case 4
			qua_vec(4) = 0.5*sqrt(s(4));
			qua_vec(1) = 0.25*(DCMbn(2,1) - DCMbn(1,2))/qua_vec(4);
			qua_vec(2) = 0.25*(DCMbn(1,3) + DCMbn(3,1))/qua_vec(4);
			qua_vec(3) = 0.25*(DCMbn(3,2) + DCMbn(2,3))/qua_vec(4);
    end
