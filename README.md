# Honours Project - Analysis of Tire Models using PhysX Engine
This honours project is centered around the implementation of 3 different tire models: Magic Formula, Dugoff and Fiala.\
The main purpose of this project is to analyse and compare these 3 models, along with NVIDIA's default tire model, to categorize them into different genres in the gaming world.
![Poster](https://github.com/Kos-hub/HonoursProjectTireSimulation/blob/main/DissertationPoster_page-0001.jpg)
## Implementing the formulas
Extensive research was conducted over the course of one year to better understand the nature of the mentioned tire models.
Once the research was completed, I familiarised myself with PhysX and its VehicleSDK and created my first vehicle snippets. Some of NVIDIA's helper classes are also imported and used in this project. \
The final product is based on the VehicleNoDrive snippet provided by NVIDIA. The reason why I picked this snippet is because by doing so, each car with a different model would follow exactly the same path,
with exactly the same torque and braking forces applied to the wheels.

## Analysing the behaviours
A CSV helper class was implemented in order to conduct a proper analysis on the tire's behaviour. This CSV helper takes into account the fact that PhysX uses substepping, which is a technique used by the engine
when the velocities in the simulation are too high, the engine will divide one physics step into multiple substeps. The CSV helper creates a temporary string for each "big" step, and whenever that step is done
only the last 4 entries in the string are grabbed from the temporary string, and the string itself is flushed, waiting for the next step in the simulation.

### Plotting data with R
An R script was written to simplify the analysis of the models. The R scripts reads the directory in which its placed, grabs all .csv files and loads them as tables. The script then sorts the first column
in descending order and, finally, uses ggplot2 to plot the data with different colours, depending on which model is currently being analysed. \
Finally, this script is called within the C++ code at the veryend of each physics simulation, completely automating the simulation and the analysis of each vehicle.
