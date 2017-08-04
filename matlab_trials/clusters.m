%pkg load statistics

function clusters()
    rdata = csvread('DataCaptures/sideNormal2.csv');

    slice_thickness = 0.45;           % Thickness of the slices
    numClusters = 30;                % Number of clusters to in a slice
    numReplicates = 2;               % Seems to work fine with 2
    distMetric = 'cityblock';        % Distance metric for k-means
    minSliceSize = numClusters*4;    % Minimum number of datapoints per slice (smaller sets are discarded)

    hold on
    rdata = rdata(1:10:end,:);        % Reduce sample rate to reduce processing time 
    
    [rdata] = normalizeCamDist(rdata);
    
    rdata = [rdata(:,1), rdata(:,3), -rdata(:,2)];
    
    % Plot the data with the color band showing
    figure(1);
    clf;
    grid on;
    axis square;
    hold on;

    for depth = 0:slice_thickness:0.45
      tic;

      % Get a slice of the data
      slice_indices = find( (rdata(:,2) > depth) & ...
                            (rdata(:,2) < (depth + slice_thickness)));
      slice = rdata(slice_indices,:);

      % Ignore slices which do not have enough points
      indSize = size(slice_indices);

      if (indSize(1) <= minSliceSize)
        continue;
      end

      toc

      % Calculate the clusters in the data
      [idx,C] = kmeans(slice,numClusters,'Distance',distMetric,'Replicates',numReplicates);

      % Plot the centroids of the clusters
      plot3(C(:,1),C(:,2),C(:,3),'b.','MarkerSize',32);
        
      adj = connectMeans(slice, idx, C);
      
      %segmentArmBand(adj, C, [-0.15 0.1 0.5]);
      
      
      for i = 1:numClusters
          clusterToPlot = slice(idx==i,:);
          clusterToPlot = clusterToPlot(1:1:size(clusterToPlot,1),:);
            
          %{
          K = convhull(clusterToPlot(:,1),clusterToPlot(:,2),clusterToPlot(:,3));

          hullPoints = clusterToPlot(K, :);

          plot3(  hullPoints(:,1), ...
                  hullPoints(:,2), ...
                  hullPoints(:,3), ...
                  '*','color',rand(1,3),'MarkerSize',8);
          %}

          plot3(  clusterToPlot(:,1), ...
                  clusterToPlot(:,2), ...
                  clusterToPlot(:,3), ...
                  '.','color',rand(1,3),'MarkerSize',8);
      end
      
      
    end
    
    hold off;
end

function [nCloud] = normalizeCamDist(cloud)
    distS = sum(cloud.^2,2);
    
    distS = distS./max(distS);
    
    nCloud = cloud(rand(size(distS)) < distS,:);
end

function [adj] = connectMeans(cloud, idx, C)
    adj = zeros(size(C,1));
    
    %cutoffDist = 0.1;
    cutoffWeight = 100;
    
    numElements = zeros(size(C,1), 1);
    
    tic
    
    % For each point in the cloud
    for j = 1:1:size(cloud,1)
        
        % Get distance squared between the point and the k means
        dist2means = sum(((C-repmat(cloud(j,:),[size(C,1) 1])).^2) , 2);
        
        % Distance between point and its cluster's mean
        dist2home = dist2means(idx(j));
        
        numElements(idx(j)) = numElements(idx(j))+ 1;
        
        % When near zero, two means are neighbors
        deltaDist = abs(dist2means-dist2home);
        
        % Close distances result in larger values
        adj(:, idx(j)) = adj(:, idx(j)) + deltaDist.^-1;
    end
    % Remove connections back to self
    adj(logical(eye(size(adj,1)))) = 0;
    
    % Remove lower part of triangle after adding it to the top half
    %adj = triu(adj) + tril(adj)';
    adj = adj+adj';
    
    adj = adj./repmat(numElements,[1 size(numElements, 1)]);
    
    % Remove sparse connections
    adj = adj > cutoffWeight;
    
    toc
    
    % For each mean (node) in pointcloud
    for node = 1:1:size(C,1)
        neighbors = C(adj(:, node),:);
        self = C(node,:);
        
        for neighbor_id = 1:1:size(neighbors,1)
            neighbor = neighbors(neighbor_id,:);
            plot3(  [neighbor(:,1), self(:,1)], ...
                    [neighbor(:,2), self(:,2)], ...
                    [neighbor(:,3), self(:,3)], ...
                    'r-','LineWidth',2);
        end
    end
end

function segmentArmBand(adj, C, hand_loc)
    epsilon = 0.0001;

    % Identify hand segment
    to_hand_dist = sum((C-repmat(hand_loc,[size(C,1) 1])).^2, 2);
    [~, cur_group_index] = min(to_hand_dist);
    
    cur_group = C(cur_group_index,:);

    % Find the global mean
    g_mean = mean(C);
    
    plot3(cur_group(:,1),cur_group(:,2),cur_group(:,3),'r.','MarkerSize',60);
    plot3(g_mean(:,1),g_mean(:,2),g_mean(:,3),'g.','MarkerSize',32);

    for k = 1:40
        neighbors = C(adj(cur_group_index,:),:);
        
        if (isempty(neighbors))
            break;
        end
        
        neighborInd = find(adj(cur_group_index,:));
        
        % Find the vectors between the current group and all groups
        rel2mean = neighbors - repmat(g_mean,[size(neighbors,1) 1]);
        dist2mean = sqrt(sum(rel2mean.^2,2));

        % Get the best-matching index
        [~, cur_group_index_new] = max(dist2mean);
        cur_group_index_new = neighborInd(cur_group_index_new);
        
        % Prevent cycles
        adj(:, cur_group_index) = 0;
        
        cur_group_index = cur_group_index_new;

        % Update the current group
        cur_group = C(cur_group_index,:);

        plot3(cur_group(:,1),cur_group(:,2),cur_group(:,3),'r.','MarkerSize',60);
        

        pause
    end
end
%{
function segmentArmBand(adj, C, hand_loc)
    epsilon = 0.0001;

    % Identify hand segment
    to_hand_dist = sum((C-repmat(hand_loc,[size(C,1) 1])).^2, 2);
    [~, cur_group_index] = min(to_hand_dist);
    start_group = C(cur_group_index,:);
    cur_group = start_group; 

    % Find the global mean
    g_mean = mean(C);
    
    plot3(g_mean(:,1),g_mean(:,2),g_mean(:,3),'g.','MarkerSize',32);
    
    % For later: remove hand index from selection
    min_dot_ind = cur_group_index;

    for k = 1:15
        % Find the terminal axis
        ax_term = g_mean-cur_group;
    
        % Find the vectors between the current group and all groups
        rel_groups = C - repmat(cur_group,[size(C,1) 1]);

        % Find the dot products between the terminal axis and the relative
        % groups. Goal: Minimize x and minimize distance to the cur_group.
        dot_to_term = dot(rel_groups, repmat(ax_term,[size(rel_groups,1) 1]), 2)
        dist_to_cur = sqrt(sum(rel_groups.^2,2));

        % Discard the current group
        dot_to_term(dot_to_term < epsilon) = 5;
        dist_to_cur(dist_to_cur < epsilon) = 5;
        
        cost = dot_to_term./dist_to_cur./norm(ax_term);
        
        C(min_dot_ind,:) = g_mean;

        % Get the best-matching index
        [~, min_dot_ind] = min(cost);%min(dot_to_term);

        % Update the current group
        cur_group = C(min_dot_ind,:);

        plot3(cur_group(:,1),cur_group(:,2),cur_group(:,3),'r.','MarkerSize',32);
        
        pause
    end
end
%}