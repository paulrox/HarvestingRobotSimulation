%% Generate the search directions for gradient estimation

file = fopen('search_d.txt', 'w');

row = zeros(1, robot.n);

for i= 1 :  2^(robot.n) - 1
    bin = dec2bin(i, robot.n);
    
    for j = 1:robot.n
        row(j) = str2double(bin(j));
        
    end
    % Normalize the direction vector
    row = row / norm(row);
    fprintf(file, '%d %d %d %d %d %d %d %d\n', row);
    fprintf(file, '%d %d %d %d %d %d %d %d\n', -row);
end

fclose(file);