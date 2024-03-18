# Wildfire -  Probabilistic RoadMap versus A* Planner

This repository contains code of a 2 player simulation comparing a sampling based Probabilistic Roadmap versus a combinatorial based A* planner. 

In the simulation, a maze-like field with trees and bushes was developed. Within this environment, an arsonist - the Wumpus - will navigate the field, setting fires while a firetruck - a Mercedes Unimog - will fight the fires. Each player will utilize a different planning approach to achieve their task. Since the Wumpus is confined to a grid, it will use a combinatorial planner, such as A*. For the Unimog, a sampling based method will be implemented, specifically a probabilistic road map. Since the Unimog utilizes standard Ackermann steering, a local planner will be implemented to move along the trajectory found by the probabilistic road map. The implementation of the local planner ensures the vehicle kinematic constraints are followed. For both players, collision detection was considered.

A full report write up for these simulations can be found in the docs folder.

## Simulation Animation

https://github.com/azzamshaikh/prm_planner/assets/47488876/e8642805-53b8-46d6-8c42-69cde4e12118

## Dependencies

The code used for the simulation was developed with Python 3.11 and Pygame 2.4. 

## Run Code

To run the simulation, from the `src` directory of this repo, run the following command. 

```
python simulation.py
```




