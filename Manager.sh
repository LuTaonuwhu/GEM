#!/bin/bash

rm output/logs/all.csv
rm output/logs/SUMO/*
rm output/logs/CWM/*

#load fcd file
#python FCDFile.py

tail -n +2 ExperimentalPlan.txt | while IFS=' ' read -r MAE K SY TD
do
printf "Executing Experiment with MAE: $MAE, K: $K, SY: $SY, TD: $TD ...\n"
python Simulate_Prediction.py -mae $MAE -pk $K -sy $SY -thd $TD
printf "Experiment Done!\n"

sumoLog=$(ls -Art output/logs/SUMO | tail -n 1)
cwmLog=$(ls -Art output/logs/CWM | tail -n 1)
sumoPath='output/logs/SUMO/'
cwmPath='output/logs/CWM/'
sumoLogPath="${sumoPath}${sumoLog}"
cwmLogPath="${cwmPath}${cwmLog}"

printf "SUMO log path: $sumoLogPath \n"
printf "CWM log path: $cwmLogPath \n"
printf "Log files processing..."
python Process_Logs.py -s $sumoLogPath -c $cwmLogPath
printf "Log files processing Done!"

printf "\n"
done