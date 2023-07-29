function T = Homogeneous(DH)
% Input: DH Table
% Output: Homogeneous matrix of the link
    a = DH(1);
    alpha = DH(2);
    d = DH(3);
    theta = DH(4);
    
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    
    T = [ct -st*ca st*sa a*ct
         st ct*ca -ct*sa a*st
         0 sa ca d
         0 0 0 1]; 
end
