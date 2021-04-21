function x = my_TLS(A,b)

[m n]   = size(A);            % (A is m by n)
Z = [A b];              % Z is A augmented with b.
[U S V] = svd(Z,0);           % find the SVD of Z.
VXY = V(1:n,1+n:end);     % Take the block of V consisting of the first n rows and the n+1 to last column
VYY = V(1+n:end,1+n:end); % Take the bottom-right block of V.
x = -VXY/VYY;

end

