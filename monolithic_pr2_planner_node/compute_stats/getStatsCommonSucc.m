function [method1_mod, method2_mod] = getStatsCommonSucc(method1, method2)
    
    succ_ind = cell(1,10);
    method1_mod = cell(1,10);
    method2_mod = cell(1,10);

    for i = 1:10

        succ_ind1 = find(~isnan(method1{i}.base));
        succ_ind2 = find(~isnan(method2{i}.base));
        
        succ_ind{i} = intersect(succ_ind1, succ_ind2);

        method1_mod{i}.base = mean(method1{i}.base(succ_ind{i}),'omitnan');
        method1_mod{i}.arm = mean(method1{i}.arm(succ_ind{i}),'omitnan');
        method1_mod{i}.time = mean(method1{i}.time(succ_ind{i}),'omitnan');
        method1_mod{i}.success = method1{i}.success;
        
        method2_mod{i}.base = mean(method2{i}.base(succ_ind{i}),'omitnan');
        method2_mod{i}.arm = mean(method2{i}.arm(succ_ind{i}),'omitnan');
        method2_mod{i}.time = mean(method2{i}.time(succ_ind{i}),'omitnan');
        method2_mod{i}.success = method2{i}.success;

        
    end
    

end