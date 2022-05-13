% Copyright 2018, Michael Otte
%
% Permission is hereby granted, free of charge, to any person obtaining a 
% copy of this software and associated documentation files (the 
% "Software"), to deal in the Software without restriction, including 
% without limitation the rights to use, copy, modify, merge, publish, 
% distribute, sublicense, and/or sell copies of the Software, and to 
% permit persons to whom the Software is furnished to do so, subject to 
% the following conditions:
%
% The above copyright notice and this permission notice shall be included 
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

% this will display the search tree and path
% assuming that the files have been generated

startIDs = [9;88;30;38;38];
goalIDs = [5;9;95;502;5002];

for index = 1:5

    input_nodes = ['inputs/nodes_',num2str(index),'.txt'];
    input_edges = ['inputs/edges_with_costs_',num2str(index),'.txt'];
    output_path = ['outputs/search_tree_',num2str(index),'.txt'];
    output_tree = ['outputs/output_path_',num2str(index),'.txt'];
    
    search_tree_raw = csvread(output_path);
    path_raw = csvread(output_tree);
    nodes_raw = csvread(input_nodes);
    edges_raw = csvread(input_edges);

    % a bit of data processing for faster plotting
    search_tree = nan(3*size(search_tree_raw, 1), 2);

    search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
    search_tree(2:3:end-1, 1) = search_tree_raw(:, 5);
    search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
    search_tree(2:3:end-1, 2) = search_tree_raw(:, 6);

    nodes = nodes_raw(2:end,2:3);

    edges_raw = edges_raw(2:end,:);

    edges = nan(3*size(edges_raw, 1), 2);

    edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
    edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
    edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
    edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);

    titlestr = ['Problem ',num2str(index),': Start Node = ',...
        num2str(startIDs(index)),', Goal Node = ',num2str(goalIDs(index))];
    image_path = ['outputs/result_image_',num2str(index),'.png'];
    
    figure(index)
    plot(nodes(:,1),nodes(:,2),'ok'); hold on
    plot(edges(:,1),edges(:,2),'k')
    plot(search_tree(:,1),search_tree(:,2),'r','LineWidth',2);
    plot(path_raw(:,2), path_raw(:,3),'g','LineWidth',3);
    plot(nodes(startIDs(index),1),nodes(startIDs(index),2),'ob',...
        'markersize',12,'linewidth',3)
    plot(nodes(goalIDs(index),1),nodes(goalIDs(index),2),'xb',...
        'markersize',12,'linewidth',3)
    hold off
    legend('','','Search Tree','Optimal Path','Starting Node',...
        'Goal Node','interpreter','latex','location','best');
    title(titlestr,'interpreter','latex','fontsize',16);
    saveas(gcf,image_path)
    
end

close all;