
for i in {0..4}; do
	
	echo "Running batch $i"

	./run_all_experiments.sh 

	mv stats stats_$i

done
