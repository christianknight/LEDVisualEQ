A = size(SOS,1);
fprintf('\nstatic int sections = %d;\n',A);
fprintf('static float coefs[] = {\n');

for i = 1:(size(SOS,1)-1)
    fprintf('\t%ef, ',SOS(i,1)*G(i));
    fprintf('%ef, ',SOS(i,2)*G(i));
    fprintf('%ef, ',SOS(i,3)*G(i));
    fprintf('%ef, ',-SOS(i,5));
    fprintf('%ef, \n',-SOS(i,6));
end
fprintf('\t%ef, ',SOS(A,1)*G(A));
fprintf('%ef, ',SOS(A,2)*G(A));
fprintf('%ef, ',SOS(A,3)*G(A));
fprintf('%ef, ',-SOS(A,5));
fprintf('%ef\n',-SOS(A,6));
fprintf('};\n\n');
