# ros2_rt_eval
```
sudo apt install git-lfs
git lfs install
git lfs track "results/*.csv"
git add .gitattributes
git add "results/*.csv"
git commit -m "Add results"
git config core.compression 9
git repack --threads=7 #for 8 threads
git push
```
on dell_g15:
```
 ./ros2_rt_eval/ros2_rt_eval/scripts/multi_cli.sh 1000000 1000000 10 ./ros2_rt_eval/results/rt_results_multi_cli/i7/
```

on i5
```
 ./src/ros2_rt_eval/ros2_rt_eval/scripts/multi_cli.sh 1000000 1000000 10 ./src/ros2_rt_eval/results/rt_results_multi_5_cli/i5/

 ./src/ros2_rt_eval/ros2_rt_eval/scripts/vec_size.sh 100000 ./src/ros2_rt_eval/results/rt_results_vec_series/i5/ 10000000
```
