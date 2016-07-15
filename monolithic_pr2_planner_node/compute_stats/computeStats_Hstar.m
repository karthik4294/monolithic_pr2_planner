clc; clear;

num = 10;

for t = 1:5

    %stats_file = ['stats60_eps10/Batch3/stats_' num2str(t-1)];    
    %stats_file = ['stats_' num2str(t-1)];
    stats_file = ['stats_omplobj/Batch1_100eps/stats_' num2str(t-1)];
    %stats_file = 'stats_4';
    
    for i = 1:num

        rrt_path = ['../' stats_file '/paths_rrt_' num2str(i-1)  '/rrt_'];
        rrtconnect_path = ['../' stats_file '/paths_rrtconnect_' num2str(i-1)  '/rrtconnect_'];
        rrtstar_path = ['../' stats_file '/paths_rrtstar_' num2str(i-1)  '/rrtstar_'];
        rrtstarfirstsol_path = ['../' stats_file '/paths_rrtstarfirstsol_' num2str(i-1)  '/rrtstarfirstsol_'];
        prm_path = ['../' stats_file '/paths_prm_' num2str(i-1)  '/prm_'];
        hstar_path = ['../' stats_file '/paths_hstar_' num2str(i-1)  '/hstar_'];
        wastar_path = ['../' stats_file '/paths_wastar_' num2str(i-1)  '/wastar_'];%hstar_path;%
 
        rrt_stats{t}{i} = computeMethodStats(rrt_path,num,0);
        rrtconnect_stats{t}{i} = computeMethodStats(rrtconnect_path,num,0);
        rrtstar_stats{t}{i} = computeMethodStats(rrtstar_path,num,0);
        rrtstarfirstsol_stats{t}{i} = computeMethodStats(rrtstarfirstsol_path,num,0);
        hstar_stats{t}{i} = computeMethodStats(hstar_path,num,0);
        prm_stats{t}{i} = computeMethodStats(prm_path,num,0);
        if(t == 1)
            wastar_stats{t}{i} = computeMethodStats(wastar_path,num,0);
        end
        
    end

end

rrt = normalizeStats(rrt_stats);
rrtconnect = normalizeStats(rrtconnect_stats);
rrtstar = normalizeStats(rrtstar_stats);
rrtstarfirstsol = normalizeStats(rrtstarfirstsol_stats);
prm = normalizeStats(prm_stats);
hstar = normalizeStats(hstar_stats);
wastar = normalizeStats(wastar_stats);

[rrt_mod, hstar_mod_rrt] = getStatsCommonSucc(rrt, hstar);
[rrtconnect_mod, hstar_mod_rrtconnect] = getStatsCommonSucc(rrtconnect, hstar);
[rrtstar_mod, hstar_mod_rrtstar] = getStatsCommonSucc(rrtstar, hstar);
[rrtstarfirstsol_mod, hstar_mod_rrtstarfirstsol] = getStatsCommonSucc(rrtstarfirstsol, hstar);
[prm_mod, hstar_mod_prm] = getStatsCommonSucc(prm, hstar);
[wastar_mod, hstar_mod_wastar] = getStatsCommonSucc(wastar, hstar);

printStats(rrt_mod, hstar_mod_rrt, 'rrt');
printStats(rrtconnect_mod, hstar_mod_rrtconnect, 'rrtconnect');
printStats(rrtstar_mod, hstar_mod_rrtstar, 'rrtstar');
printStats(rrtstarfirstsol_mod, hstar_mod_rrtstarfirstsol, 'rrtstarfirstsol');
printStats(prm_mod, hstar_mod_prm, 'prm');
printStats(wastar_mod, hstar_mod_wastar, 'wastar');








    
    





