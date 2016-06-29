function [rrt, rrtconnect, hstar, wastar] = normalizeStats(rrt_stats, rrtconnect_stats, hstar_stats, wastar_stats)

    temp_rrt_base = cell(1,10);
    temp_rrt_arm = cell(1,10);
    temp_rrt_time = cell(1,10);
    temp_rrt_succ = cell(1,10);

    temp_rrtconnect_base = cell(1,10);
    temp_rrtconnect_arm = cell(1,10);
    temp_rrtconnect_time = cell(1,10);
    temp_rrtconnect_succ = cell(1,10);

    temp_hstar_base = cell(1,10);
    temp_hstar_arm = cell(1,10);
    temp_hstar_time = cell(1,10);
    temp_hstar_succ = cell(1,10);

    temp_wastar_base = cell(1,10);
    temp_wastar_arm = cell(1,10);
    temp_wastar_time = cell(1,10);
    temp_wastar_succ = cell(1,10);

    for j = 1:10

        for i = 1:3

            temp_rrt_base{j} = [temp_rrt_base{j};rrt_stats{i}{j}.base];
            temp_rrt_arm{j} = [temp_rrt_arm{j};rrt_stats{i}{j}.arm_sqr];
            temp_rrt_time{j} = [temp_rrt_time{j};rrt_stats{i}{j}.time];
            temp_rrt_succ{j} = [temp_rrt_succ{j};rrt_stats{i}{j}.success];

            temp_rrtconnect_base{j} = [temp_rrtconnect_base{j};rrtconnect_stats{i}{j}.base];
            temp_rrtconnect_arm{j} = [temp_rrtconnect_arm{j};rrtconnect_stats{i}{j}.arm_sqr];
            temp_rrtconnect_time{j} = [temp_rrtconnect_time{j};rrtconnect_stats{i}{j}.time];
            temp_rrtconnect_succ{j} = [temp_rrtconnect_succ{j};rrtconnect_stats{i}{j}.success];

            temp_hstar_base{j} = [temp_hstar_base{j};hstar_stats{i}{j}.base];
            temp_hstar_arm{j} = [temp_hstar_arm{j};hstar_stats{i}{j}.arm_sqr];
            temp_hstar_time{j} = [temp_hstar_time{j};hstar_stats{i}{j}.time];
            temp_hstar_succ{j} = [temp_hstar_succ{j};hstar_stats{i}{j}.success];

        end

        temp_wastar_base{j} = [temp_wastar_base{j};wastar_stats{1}{j}.base];
        temp_wastar_arm{j} = [temp_wastar_arm{j};wastar_stats{1}{j}.arm_sqr];
        temp_wastar_time{j} = [temp_wastar_time{j};wastar_stats{1}{j}.time];
        temp_wastar_succ{j} = [temp_wastar_succ{j};wastar_stats{1}{j}.success];

    end

    rrt = cell(1,10);
    rrtconnect = cell(1,10);
    hstar = cell(1,10);
    wastar = cell(1,10);

    for i = 1:10

        rrt{i}.base = mean(temp_rrt_base{i},1,'omitnan');
        rrt{i}.arm = mean(temp_rrt_arm{i},1,'omitnan');
        rrt{i}.time = mean(temp_rrt_time{i},1,'omitnan');
        rrt{i}.success = mean(temp_rrt_succ{i},1,'omitnan');

        rrtconnect{i}.base = mean(temp_rrtconnect_base{i},1,'omitnan');
        rrtconnect{i}.arm = mean(temp_rrtconnect_arm{i},1,'omitnan');
        rrtconnect{i}.time = mean(temp_rrtconnect_time{i},1,'omitnan');
        rrtconnect{i}.success = mean(temp_rrtconnect_succ{i},1,'omitnan');

        hstar{i}.base = mean(temp_hstar_base{i},1,'omitnan');
        hstar{i}.arm = mean(temp_hstar_arm{i},1,'omitnan');
        hstar{i}.time = mean(temp_hstar_time{i},1,'omitnan');
        hstar{i}.success = mean(temp_hstar_succ{i},1,'omitnan');

        wastar{i}.base = mean(temp_wastar_base{i},1,'omitnan');
        wastar{i}.arm = mean(temp_wastar_arm{i},1,'omitnan');
        wastar{i}.time = mean(temp_wastar_time{i},1,'omitnan');
        wastar{i}.success = mean(temp_wastar_succ{i},1,'omitnan');

    end

end