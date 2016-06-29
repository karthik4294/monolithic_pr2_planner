function stats = computeMethodStats(folder_name,num,sbpl)
  base = [];
  spine = [];
  obj = [];
  arm_sqr = [];
  arm_abs = [];
  time = [];
  cost = [];
  expands = [];
  count = 0;
  for i=1:num
    path_filename = [folder_name num2str(i-1,'%02d') '.path'];
    stat_filename = [folder_name num2str(i-1,'%02d') '.stats'];
    if exist(path_filename, 'file')~=2 || exist(stat_filename, 'file')~=2
      base(i) = NaN;
      spine(i) = NaN;
      obj(i) = NaN;
      arm_abs(i) = NaN;
      arm_sqr(i) = NaN;
      time(i) = NaN;
      cost(i) = NaN;
      expands(i) = NaN;
      continue;
    end
    count = count + 1;
    raw_path = load(path_filename);
    %read 'sub-trajectories' that make up the full path
    base_path = raw_path(:,1:3);
    spine_path = raw_path(:,4);
    arm_path = raw_path(:,5:11);
    obj_path = raw_path(:,12:15);
    %figure;plot(base_path(:,1), base_path(:,2))

    base(i) = sum(sqrt(sum(diff(base_path(:,1:2)).^2,2)));
    spine(i) = sum(sqrt(sum(diff(spine_path).^2,2)));
    obj(i) = sum(sqrt(sum(diff(obj_path(:,1:3)).^2,2)));

    %compute arm path
    arm_diff = [];
    for j=2:size(arm_path,1)
       for k=1:size(arm_path,2)
         ang1 = mod(arm_path(j-1,k), 2*pi);
         ang2 = mod(arm_path(j,k), 2*pi);
         ang = mod(ang2-ang1, 2*pi);
         if ang > pi
           ang = -(2.0*pi - ang);
         end
         arm_diff(j-1,k) = ang;
       end
    end
    arm_abs(i) = sum(sqrt(sum(abs(arm_diff),2)));
    arm_sqr(i) = sum(sqrt(sum(arm_diff.^2,2)));

    %get plan times
    raw_stats = load(stat_filename);
    if sbpl
      time(i) = raw_stats(1,1);
      cost(i) = raw_stats(1,9);
      expands(i) = raw_stats(1,8);
    else
      time(i) = raw_stats(1,1) + raw_stats(1,2);
      cost(i) = 0;
      expands(i) = 0;
    end
  end
  stats.name = folder_name;
  stats.base = base;
  stats.spine = spine;
  stats.obj = obj;
  stats.arm_abs = arm_abs;
  stats.arm_sqr = arm_sqr;
  stats.time = time;
  stats.success = count;
%   % if sbpl
%       stats.cost = cost;
%       stats.expands = expands;
%   % end

end

