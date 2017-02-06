
for i in {0..4}; do
	
	echo "Running batch $i"

	./run_all_experiments_ompl.sh 

	mv stats stats_bitstarfirstsol_$i

done


