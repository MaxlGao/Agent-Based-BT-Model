# Social Insect-Inspired Behaviors for Collective Search Operations by Unmanned Aerial Vehicle (UAV) Swarms
Python Repository for UAV Swarm Model. 

## Simulation Configuration
Configure simulations by editing MaxsLaw.py, or create your own configuration file. If you make your own file, change the import settings in masterScript. 

Here are some settings in MaxsLaw: 
- multiLaw: Setting this to True enables use of MultiLaw.py, setting up for parallel processing of many simulations. Setting this to False will run a number of simulations equal to numSims, and render it.
- renderFlavor: Enables more shapes to be drawn if rendering is enabled.
- printProgress: Prints simulation status every 100 time steps. 
- saveData: if True, saves output data to a .csv file.
- debugEveryX: Enables printing of messages every time a node in the behavior tree is activated. Setting these to True will do nothing as of now.

## Simulation Structure
Running masterScript is required to execute simulations. masterScript makes a call to a run_simulations script in Simulator.py, which in turn initializes a Swarm object detailed in Swarm.py. 
run_simulations calls a stepSimulation method in the Swarm class, which iterates through all Agent objects in Swarm. 
An Environment object, detailed in Environment.py is used in Swarm, includes all Target, Hazard, and Home objects. 
Finally, Utilities.py contains many useful functions commonly used by other scripts. 