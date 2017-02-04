eps = ['1.5', '2', '3', '5', '10', '50'];

wa_rt_H = [0.44, 0.27, 0.27, 0.27, 0.26, 0.25];
h_rt_H = [0.43, 0.26, 0.24, 0.22, 0.22, 0.19];

wa_rt_Passage = [0.94, 0.32, 0.30, 0.30, 0.31, 0.28];
h_rt_Passage = [0.82, 0.28, 0.21, 0.18, 0.17, 0.15];

wa_rt_Maze = [2.50, 1.39, 1.24, 0.96, 0.90, 0.85];
h_rt_Maze = [3.33, 1.84, 1.44, 1.24, 1.19, 1.00];


bar([wa_rt_H' h_rt_H']);%, 'LineWidth', 2);
Labels = {1.5, 2, 3, 5, 10, 50};
set(gca, 'XTick', 1:6, 'XTickLabel', Labels);
%hold on
%bar(eps, h_rt_Maze, 'g');%, 'LineWidth', 2);
xlabel('Epsilon w','fontweight','bold','fontsize',16);
ylabel('Run time(s)','fontweight','bold','fontsize',16);
%title(['Runcomparison of WA* and H* for '])
legend('WA*','H*');