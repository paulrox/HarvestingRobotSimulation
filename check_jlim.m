function [ ret ] = check_jlim( r, q )
%CHECK_JLIM Check if the actual pose exceeds the joints limits
%   It uses the 'islimit' function of the 'SerialLink' class to check
%   which joints exceeded its limits and prints out this information.

    limits = r.islimit(q);
    limits = limits(:,1);
    
    if any(limits)
        % At least one joint exceed its limits
        ret = 1;
        low_lim = find(limits < 0);
        for i = 1:size(low_lim)
            disp(['Joint ' num2str(low_lim(i)) ' is at ' ...
                num2str(q(low_lim(i))) ' but its lower limit is ' ...
                num2str(r.qlim(low_lim(i),1))]);
        end
        
        up_lim = find(limits > 0);
        for i = 1:size(up_lim)
            disp(['Joint ' num2str(up_lim(i)) ' is at ' ...
                num2str(q(up_lim(i))) ' but its upper limit is ' ...
                num2str(r.qlim(up_lim(i),2))]);
        end
    end
end

