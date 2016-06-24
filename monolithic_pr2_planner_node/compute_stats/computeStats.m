function comparison = computeStats(path)
  num = 3;

  % prm_stats = computeMethodStats('/prm_',num,0)
  rrt_stats = computeMethodStats('paths_rrt_4/rrt_',num,0)
  rrtconnect_stats = computeMethodStats('paths_rrtconnect_4/rrtconnect_',num,0)
  %ara_stats = computeMethodStats([path '/ara_'],num,1)
  %imha_stats = computeMethodStats([path '/imha_'],num,1)
  %smha_stats = computeMethodStats([path '/smha_'],num,1)
  %mpwa_stats = computeMethodStats([path '/mpwa_'],num,1)
  %mhg_reex_stats = computeMethodStats([path '/mhg_reex_'],num,1)
  %mhg_no_reex_stats = computeMethodStats([path '/mhg_no_reex_'],num,1)
  %ees_stats = computeMethodStats([path '/ees_'],num,1)
  hstar_stats = computeMethodStats('paths_hstar_4/hstar_',num,0)
  wastar_stats = computeMethodStats('paths_wastar_4/wastar_',num,0)

  other_methods = [wastar_stats rrt_stats rrtconnect_stats] ;
  %other_methods = [cbirrt_stats multi_ompl_stats];

  comparison = compareMethods(hstar_stats,other_methods);

  %displayComparison(comparison);
end
