function delta = transformation_diff(A, B)


    % handle arguments, convert all to 4x4 matrices
    if nargin > 0
        if isa(A, 'SE3')  delta = [ tform2trvec(TD) tform2eul(TD) ]' ;
            T1 = A.double;
        elseif ishomog(A)            
            T1 = A;
        else
            error('RTB:tr2delta:badarg', 'T1 should be a homogeneous transformation');
        end
        T0 = eye(4,4);
    end

    if nargin > 1
        T0 = T1;
        if isa(B, 'SE3')
            T1 = B.double;
        elseif ishomog(B)
            T1 = B;
        else
                error('RTB:tr2delta:badarg', 'T0 should be a homogeneous transformation');
        end
    end
    
    % compute incremental transformation from T0 to T1 in the T0 frame
    TD = inv(T0) * T1;

    delta = [ vex(TD(1:3,1:3)-eye(3,3))' tform2trvec(TD)  ]' ;
    % build the delta vector
%    delta = [transl(TD); vex(t2r(TD) - eye(3,3))];
    
     
%    R0 = t2r(T0); R1 = t2r(T1);
%    % in world frame
%    %[th,vec] = tr2angvec(R1*R0');
%    dR = vex(R1*R0');
%    %delta = [ (T1(1:3,4)-T0(1:3,4)); th*vec' ];
%    delta = [ (T1(1:3,4)-T0(1:3,4)); dR];

% same as above but more complex
    delta = [	...
        0.5*(	cross(T0(1:3,1), T1(1:3,1)) + ...
            cross(T0(1:3,2), T1(1:3,2)) + ...
            cross(T0(1:3,3), T1(1:3,3)) ...
        ) ;
        T1(1:3,4)-T0(1:3,4)
        
        ];
end


function h = ishomog(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  all(d(1:2) == [4 4]);

        if h && nargin > 1
            h = abs(det(tr(1:3,1:3)) - 1) < eps;
        end

    else
        h = false;
    end
end


function v = vex(S)
    if all(size(S) == [3 3])
        v = 0.5*[S(3,2)-S(2,3); S(1,3)-S(3,1); S(2,1)-S(1,2)];
    elseif all(size(S) == [2 2])
        v = 0.5*(S(2,1)-S(1,2));
    else
        error('RTB:vex:badarg', 'argument must be a 2x2 or 3x3 matrix');
    end
end
