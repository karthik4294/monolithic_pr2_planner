function [method] = normalizeStats(method_stats)

    temp_method_base = cell(1,10);
    temp_method_arm = cell(1,10);
    temp_method_time = cell(1,10);
    temp_method_succ = cell(1,10);

    for j = 1:10

        for i = 1:size(method_stats,2)

            temp_method_base{j} = [temp_method_base{j};method_stats{i}{j}.base];
            temp_method_arm{j} = [temp_method_arm{j};method_stats{i}{j}.arm_sqr];
            temp_method_time{j} = [temp_method_time{j};method_stats{i}{j}.time];
            temp_method_succ{j} = [temp_method_succ{j};method_stats{i}{j}.success];

        end

    end

    method = cell(1,10);

    for i = 1:10

        method{i}.base = mean(temp_method_base{i},1,'omitnan');
        method{i}.arm = mean(temp_method_arm{i},1,'omitnan');
        method{i}.time = mean(temp_method_time{i},1,'omitnan');
        method{i}.success = mean(temp_method_succ{i},1,'omitnan');

    end

end
