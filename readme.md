Multi-robot task allocation for safe planning against stochastic hazard dynamics
====
This study is available at [arxiv](https://arxiv.org/abs/2103.01840), furthermore the original Master Thesis work can be found in the [ETH Research Collection](https://www.research-collection.ethz.ch/handle/20.500.11850/471229).

**Repository content**
This repository contains the implementation of the two-stage framework introduced in the paper extended with a framework for generating random maps and save statistical results. We implemented the framework in python (see files in the [two_stage_framework](https://github.com/TihanyiD/multi_alloc/tree/master/two_stage_framework) folder). This folder contains two subfolders:

* The [two_stage_framework_case_studies](https://github.com/TihanyiD/multi_alloc/tree/master/two_stage_framework/two_stage_framework_case_studies) folder) for examining existing case studies and generating new ones.

* The [two_stage_framework_random_maps](https://github.com/TihanyiD/multi_alloc/tree/master/two_stage_framework/two_stage_framework_random_maps) folder) for generating random maps and statistics given certain hyperparameters and a fixed map structure.

Overview
----

The purpose of the framework is to generate control policies for the robots which maximizes the probability of avoiding the evolving hazard while visiting the known target locations. This requires both the allocation of targets between the robots and the generation of paths for them. 

By using the implementation presented in this repository, one can create new example environments and obtain solutions for them, or plot the pre-computed results of existing case studies. Furthermore, one can also generate random maps and statistics with given hyperparameters for a fixed map structure.

Examining existing case studies
----

Follow the instructions below:

* Open the ["two_stage_framework.py"](https://github.com/TihanyiD/multi_alloc/blob/master/two_stage_framework/two_stage_framework_case_studies/two_stage_framework.py) file.

* Make sure that the following flag is set to "True".

```
open_case_study=True
```

* Choose the case study by modifying the following line of code. Available case studies: "case_study_1", "example_2_1", "example_2_2", "example_3_1", "example_3_2".

```
example_name="case_study_1"
```

* Run the code.

The code generates the map of the example with the initial state of the hazard sources and the robots. The code then loads the pre-computed results. They are shown in the command line. The generated robot paths are shown in figures.

Creating a new example
----

Follow the instructions below:

* Open the ["two_stage_framework.py"](https://github.com/TihanyiD/multi_alloc/blob/master/two_stage_framework/two_stage_framework_case_studies/two_stage_framework.py) file.

* Make sure that the following flag is set to "False".

```
open_case_study=False
```

* Choose the name of the example you want to create by modifying the following line of code.

```
example_name="case_study_1"
```

* Set up the map of the example by modifying the following line of code. The variable "parameters.map" should be a 2 dimensional binary "numpy" array where an entry of 1 represents an obstacle in the environment.

```
parameters.map=np.array([[1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1],
                         [1,0,0,0,0,0,1,1,0,1,0,0,1,0,0,0,1],
                         [1,0,1,0,1,0,1,1,0,1,0,0,0,0,1,0,1],
                         [1,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1],
                         [1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,1,1],
                         [1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,1],
                         [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1],
                         [1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,1],
                         [1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,1],
                         [1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],
                         [1,0,1,0,1,0,1,1,0,0,1,0,0,0,0,0,1],
                         [1,0,0,0,0,0,1,1,0,0,1,0,0,0,0,1,1],
                         [1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,1]])
```

* Set up the targets by modifying the following lines of code. The variable "parameters.targets" should be a list containing tuples of coordinates representing the target locations on the map. A valid coordinate should be within the range of the map and not conside with an obstacle. The variable "parameters.task_ids" should be a list of strings containing target labels.

```
parameters.targets=[(3,9),(5,1),(8,6),(11,11),(14,1)]
parameters.task_ids=["i","ii","iii","iv","v"]
```

* Set up the robots by modifying the following lines of code. The variable "parameters.robot_positions" should be a list containing tuples of coordinates representing the initial robot locations on the map. A valid coordinate should be within the range of the map and not conside with an obstacle. Variables "parameters.robot_ids" and "parameters.robot_linestyles" should be lists containing robot labels and path line styles.

```
parameters.robot_positions=[(0,6),(8,12),(10,0)]
parameters.robot_ids=["1","2","3"]
parameters.robot_linestyles=[(0,()),(0,(3,3)),(0,(1,2))]
```

* Set up the hazards by modifying the following lines of code. The variable "parameters.y_0" should be a list of lists, where each sub-list contains tuples of coordinates representing the initial state of a particular hazard source. Multiple cell coordinates can be added for each hazard source. A valid coordinate should be within the range of the map and not conside with an obstacle. Variables "parameters.hazard_ids" and "parameters.p_f" should be lists containing hazard labels and spreading speed parameters (float between 0 and 1 -- the higher the value the higher the change for the hazard to spread to a neighbouring cell).

```
parameters.y_0=[[(13,12)],[(2,1)],[(11,2)],[(3,11)],[(13,6)]]
parameters.hazard_ids=["a","b","c","d","e"]
parameters.p_f=[0.002,0.004,0.012,0.012,0.012]
```

* Set up the goal positionby modifying the following line of code. The variable "parameters.goal" should be a tuple representing the coordinate of the goal position. A valid coordinate should be within the range of the map and not conside with an obstacle.

```
parameters.goal=(16,9)
```

* Set up the number of Monte-Carlo samples used to approximate the hazard spread. The variable "parameters.E" should be an integer representing the number of Monte-Carlo samples.
```
parameters.E=5000
```

* Set up the number of time steps for the dynamic programming algorithm. The variable "parameters.N" should be an integer representing the number of time steps.
```
parameters.N=75
```

* Set variable "parameters.p_stay=0" for a deterministic robot dynamics. Set a value between 0 and 1 for a non-deterministic robot dynamics. For example, if "parameters.p_stay=0.2", then the robot dynamics is defined as follows: the robot fails to move to the neighboring cells despite the control input and stays on the same cell with probability 0.2.
```
parameters.p_stay=0
```

* Run the code.

The code generates the map of the example with the initial state of the hazard sources and the robots (make sure to hid "close" for the program to move on). The code then loads the pre-computed results. They are shown in the command line. The generated robot paths are shown in figures.

Generating random maps and statistics
----

Follow the instructions below:

* Open the ["two_stage_framework.py"](https://github.com/TihanyiD/multi_alloc/blob/master/two_stage_framework/two_stage_framework_random_maps/two_stage_framework.py) file.

* Focus on the parameters between lines 257 and 264.

```
# Parameters
n_samples=20
n_repeat_samples=5    
n_targets=2
n_robots=2
n_hazards=3
hazard_p_f=0.02
open_case_study=False
```

* Make sure that the following flag is set to "False".

```
open_case_study=False
```

* The following variables can be modified to generate random samples and statistics: "n_samples" - the number of generated random maps; "n_repeat_samples" - in case the generated samples is invalid (this occurs if the map is impossible to solve even with the perfect strategy) the random map is regenerated this many times; "n_targets" - the number of randomly placed targets (must be between 1 and 8); "n_robots" - the number of randomly placed robots (must be between 1 and 5); "n_hazards" - the number of randomly placed hazard sources (must be between 1 and 4); and "hazard_p_f" - the common spread speed parameter of the hazard sources (must be between 0 and 1).

* The statistics are saved in a table named "results_table.csv". The table contains the overall calculation time data in seconds and the optimal success probability values for each algorithm and each sample.

Contact
----

Feel free to contact me for any queries via email.

Daniel Tihanyi

<cite>[Automatic Control Laboratory (IfA)]<cite>

ETH Z&uuml;rich

E-mail: [tihanyid@control.ee.ethz.ch](tihanyid@control.ee.ethz.ch)


[Automatic Control Laboratory (IfA)]: http://control.ee.ethz.ch/
