## GEM: a generalized error model for vehicle trajectory prediction
A generalized error model (GEM) for vehicle trajectory prediction (VTP).

This is the source code of our VTP simulation system proposed in our paper: "How Accurate Should Prediction Be for Connected Autonomous Vehicles at Blind Intersections?" 

GEM is able to simulate a VTP process with fully-controllable errors.

## Install

`git clone https://github.com/LuTaonuwhu/GEM.git`

## Usage
- make output directory
  
  `mkdir -p output/logs/CWM`

  `mkdir -p output/logs/SUMO`

### Batch Execution
- Generate FCD file:

  `python Simulate_Prediction.py -g -f /the/saving/path/to/FCD/file`

- Dump FCD file:
  
  `python FCDFile.py -f /path/to/your/FCD/file -d /path/for/dumping/FCD/file`

- Arrange your experiment plan:

  specify all the hyperparameter you want to test in ./ExperimentalPlan.txt

- Run all these experiments through the Manager

  `./Manager.sh` 

### Single Test
  
  `python Simulate_Prediction.py -k -f -s -g -i -mae -k -pk -sy -thd -track`

#### Parameter explanations:

  `python Simulate_Prediction.py --help`

## Dependency
- SUMO
- Matplotlib
- Shapely



