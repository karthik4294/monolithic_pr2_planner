function printStats(method1, method2, method_name)
    
    file_name = cell(1,10);
    base1 = [];base2 = [];
    arm1 = [];arm2 = [];
    time1 = [];time2 = [];
    succ1 = [];succ2 = [];

    for i = 1:10
        
         %file_name{i} = [method_name num2str(i-1) '.txt'];
         %fileID = fopen(file_name{i}, 'w');
        
         %fprintf(fileID, '%s %s %0.3f %s %0.3f %s %0.3f %s %0.1f%s\n',method_name,'&',method1{i}.base, ...
         %                 '&',method1{i}.arm,'&',method1{i}.time,'&',(method1{i}.success)*10,'\\');
         %fprintf(fileID, '%s %s %0.3f %s %0.3f %s %0.3f %s %0.1f%s\n','hstar','&',method2{i}.base, ...
         %                 '&',method2{i}.arm,'&',method2{i}.time,'&',(method2{i}.success)*10,'\\');             
             
        base1 = [base1;method1{i}.base];
        base2 = [base2;method2{i}.base];
        
        arm1 = [arm1;method1{i}.arm];
        arm2 = [arm2;method2{i}.arm];
        
        time1 = [time1;method1{i}.time];
        time2 = [time2;method2{i}.time];
       
        succ1 = [succ1;method1{i}.success*10];
        succ2 = [succ2;method2{i}.success*10];
        
                     
    end
    
    file_name_avg = [method_name '.txt'];
    fileID = fopen(file_name_avg, 'w');

    fprintf(fileID, '%s %s %0.3f %s %0.3f %s %0.3f %s %0.1f%s\n',method_name,'&',mean(base1,'omitnan')/mean(base2,'omitnan'), ...
                     '&',mean(arm1,'omitnan')/mean(arm2,'omitnan'),'&',mean(time1,'omitnan')/mean(time2,'omitnan'),'&',mean(succ1,'omitnan'),'\\');
    fprintf(fileID, '%s %s %0.3f %s %0.3f %s %0.3f %s %0.1f%s\n','hstar','&',mean(base2,'omitnan'), ...
                     '&',mean(arm2,'omitnan'),'&',mean(time2,'omitnan'),'&',mean(succ2,'omitnan'),'\\');  
       

end
