fprintf('[');

for i=0:1:2^(robot.n)-1
     bin = dec2bin(i, robot.n);
   
    for j = 1:robot.n
       
        fprintf('%s ', bin(j));
    end
        fprintf(';\n');
        
end

fprintf(']');