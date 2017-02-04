
for i in {0..4}; do
	
	echo "Running batch $i"

	./run_all_experiments_ompl.sh 

	mv stats stats_rrtstar_$i

done


