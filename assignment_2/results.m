% Start locations and goal regions given in problems
start = [0,0;27,30;45,-45;-16,10;39,5];
goal = [-38,20,10;-48,20,10;-45,45,15;18,-45,5;38,-8,3];

% Given obstables
obs_txt = 'obstacles.txt';
obs = csvread(obs_txt);
num_obs = obs(1,1);

% Show each problem
for ii = 1:5
    
    % Read output path and search tree files
    pathfile = ['outputs/output_path_',num2str(ii),'.txt'];
    treefile = ['outputs/output_tree_',num2str(ii),'.txt'];
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
    
    % Plot line segments in search tree output file
    for jj = 1:num_edges
        plot([tree(jj,1),tree(jj,3)],[tree(jj,2),tree(jj,4)],...
        'b','Linewidth',2);
    end
    
    % Show start location and goal region
    scatter(start(ii,1),start(ii,2),200,'xr','Linewidth',3);
    goal_circle(goal(ii,:));
    
    % Plot path in output file
    plot(path(:,1),path(:,2),'--r','Linewidth',2);
    
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
