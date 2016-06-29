clc; clear;

num = 10;

for t = 1:3

    stats_file = ['stats_' num2str(t-1)];    
    
    for i = 1:num

        rrt_path = ['../' stats_file '/paths_rrt_' num2str(i-1)  '/rrt_'];
        rrtconnect_path = ['../' stats_file '/paths_rrtconnect_' num2str(i-1)  '/rrtconnect_'];
        hstar_path = ['../' stats_file '/paths_hstar_' num2str(i-1)  '/hstar_'];
        wastar_path = ['../' stats_file '/paths_wastar_' num2str(i-1)  '/wastar_'];
 
        rrt_stats{t}{i} = computeMethodStats(rrt_path,num,0);
        rrtconnect_stats{t}{i} = computeMethodStats(rrtconnect_path,num,0);
        hstar_stats{t}{i} = computeMethodStats(hstar_path,num,0);
        if(t == 1)
            wastar_stats{t}{i} = computeMethodStats(wastar_path,num,0);
        end
        
    end

end

[rrt, rrtconnect, hstar, wastar] = normalizeStats(rrt_stats, rrtconnect_stats, hstar_stats, wastar_stats);

[rrt_mod, hstar_mod_rrt] = getStatsCommonSucc(rrt, hstar);
[rrtconnect_mod, hstar_mod_rrtconnect] = getStatsCommonSucc(rrtconnect, hstar);
[wastar_mod, hstar_mod_wastar] = getStatsCommonSucc(wastar, hstar);

printStats(rrt_mod, hstar_mod_rrt, 'rrt');
printStats(rrtconnect_mod, hstar_mod_rrtconnect, 'rrtconnect');
printStats(wastar_mod, hstar_mod_wastar, 'wastar');








    
    





