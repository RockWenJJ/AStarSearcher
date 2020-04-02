 function map = obstacle_map(start_pt,target_pt,SIZE_X,SIZE_Y)
%This function returns a map contains random distribution obstacles.
    map = zeros(SIZE_X, SIZE_Y);
    % 0-open_space, 1-start, 2-target, 3-obstacle
    map(start_pt(1,1), start_pt(1,2)) = 1;
    map(target_pt(1,1), target_pt(1,2)) = 2;
    rand_map = rand(SIZE_X,SIZE_Y);
    obstacle_ratio = 0.25;
    for i = 1:1:SIZE_X
        for j = 1:1:SIZE_Y
            if( (rand_map(i,j) < obstacle_ratio) && (i~= start_pt(1,1) || j~=start_pt(1,2)) && (i~= target_pt(1) || j~=target_pt(2)))
                map(i,j) = 3;
            end    
        end
    end
end

