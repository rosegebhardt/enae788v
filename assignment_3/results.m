% Display results of RRT search with dynamics and volume
clearvars; close all; clc;

% Start locations and goal regions given in problems
start = [30,-35;40,-40;30,40;-50,-30;-30,-35];
goal = [0,0,5;30,40,5;-50,-30,5;30,-35,5;-35,30,5];

% Given obstables
obs_txt = 'obstacles.txt';
obs = csvread(obs_txt);
num_obs = obs(1,1);

% Show each problem
for ii = 1:5
    
    trajfile = ['outputs/output_traj_',num2str(ii),'.txt'];
    pathfile = ['outputs/output_path_',num2str(ii),'.txt'];
    treefile = ['outputs/output_tree_',num2str(ii),'.txt'];
    
    traj = csvread(trajfile);
    path = csvread(pathfile);
    tree = csvread(treefile);

    [num_edges,~] = size(tree);
    
    % Image output path
    image_path = ['outputs/result_image_',num2str(ii),'.png'];
    
    % Figure titles
    title_str = ['Problem ',num2str(ii)];
    
    % Plot path and search tree
    fig = figure(ii);
    set(fig,'units','normalized','outerposition',[0.1 0.1 0.5 0.8])

    % Plot obstacles
    for jj = 2:1+num_obs
        obs_circle(obs(jj,:)); 
        hold on;
    end
    
    % Plot search graph nodes and trajectories
    for jj = 1:num_edges
        scatter(tree(jj,2),tree(jj,3),20,'b','filled');
    end
    plot(traj(1,:),traj(2,:),'b','Linewidth',1)
    
    % Plot feasible path found
    plot(path(1,:),path(2,:),'r','Linewidth',2)
    
    % Show start location and goal region
    scatter(start(ii,1),start(ii,2),200,'xr','Linewidth',3);
    goal_circle(goal(ii,:));
    
    % Plot settings
    hold off;
    axis equal;
    xlim([-50,50]); ylim([-50,50]);
    title(title_str,'interpreter','latex','fontsize',18);
    
    % Save image as .png file
    saveas(gcf,image_path)
end

% Fill in black circles at obstacles
function h = obs_circle(obs_ii)
    theta = linspace(0,2*pi,500);
    x = obs_ii(3)*cos(theta)+obs_ii(1);
    y = obs_ii(3)*sin(theta)+obs_ii(2);
    h = fill(x,y,'k','Linewidth',2);
end

% Empty red circles at goal regions
function h = goal_circle(goal_ii)
    theta = linspace(0,2*pi,500);
    x = goal_ii(3)*cos(theta)+goal_ii(1);
    y = goal_ii(3)*sin(theta)+goal_ii(2);
    h = plot(x,y,'r','Linewidth',2);
end
