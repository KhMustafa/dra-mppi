
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

plot_roadmap = True
write_to_maps = False


# Define the parameters of the step function
amplitude = 35
step_size = 35 
period = 80
length_of_spline = 600
distance_between_nodes = 10


# Define the range of x values
x_coordinates = np.arange(0, length_of_spline, distance_between_nodes)

y_coordinates = []
theta_coordinates = -np.pi/2.0*np.ones(len(x_coordinates))

for x in x_coordinates:
    x_shift = x // period
    theta = (-1) ** x_shift
    y_coordinates.append(amplitude * (theta * (x % period > step_size)))

y_coordinates = np.array(y_coordinates)




file_content = """\
<?xml version="1.0" encoding="UTF-8"?>
<!-- 1.5708 = pi / 2 -->
<!-- 
uint8 LANECENTER_FREEWAY=1
uint8 LANECENTER_SURFACESTREET=2
uint8 LANECENTER_BIKELANE=3
uint8 ROADLINE_BROKENSINGLEWHITE=6
uint8 ROADLINE_SOLIDSINGLEWHITE=7
uint8 ROADLINE_SOLIDDOUBLEWHITE=8
uint8 ROADLINE_BROKENSINGLEYELLOW=9
uint8 ROADLINE_BROKENDOUBLEYELLOW=10
uint8 ROADLINE_SOLIDSINGLEYELLOW=11
uint8 ROADLINE_SOLIDDOUBLEYELLOW=12
uint8 ROADLINE_PASSINGDOUBLEYELLOW=13
uint8 ROADEDGEBOUNDARY=15
uint8 ROADEDGEMEDIAN=16
uint8 STOPSIGN=17
uint8 CROSSWALK=18
uint8 SPEEDBUMP=19 
-->
<way>\
"""

if plot_roadmap:
    roadmap_plot = plt.plot(x_coordinates,y_coordinates, 'b')
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    roadmap_plot = plt.grid()

for index, (x, y, theta) in enumerate(zip(x_coordinates,y_coordinates,theta_coordinates)):

    file_content += f'\n  <nd x="{x}" y="{y}" theta="{theta+np.pi/2}"/>'

    if plot_roadmap:
        if index == 0:
            color = 'g'
        elif index == len(theta_coordinates)-1: 
            color = 'r'
        else:
            color = 'b'
        roadmap_plot = plt.plot(x, y, color, marker=(3, 0, theta*180/np.pi), markersize=10, linestyle='None')
        
file_content += """
  <lane type="road" width="4.0" two_way="0"/>
</way>
"""

if write_to_maps:
    file_name = f"simple_steps_{str(step_size)}"
    path = Path(__file__).parent
    file = open(f"{path}/../maps/{file_name}.xml", "w")
    file.write(file_content)
    file.close()

if plot_roadmap:
    plt.show()