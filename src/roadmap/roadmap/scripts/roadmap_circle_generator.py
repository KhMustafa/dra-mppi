
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

plot_roadmap = True
write_to_maps = False

turnarounds = 1
nodes_per_turnaround = 20
theta_intervals = 2*np.pi / nodes_per_turnaround
radius_circle = 45

spline_length = turnarounds * np.pi*2

distance_on_spline = np.arange(0, spline_length, theta_intervals)

x_coordinates = np.sin(distance_on_spline)*radius_circle
y_coordinates = np.cos(distance_on_spline)*radius_circle-radius_circle
theta_coordinates = -distance_on_spline - np.pi/2

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
    file_name = f"simple_circle_radius_{str(radius_circle)}"
    path = Path(__file__).parent
    file = open(f"{path}/../maps/{file_name}.xml", "w")
    file.write(file_content)
    file.close()

if plot_roadmap:
    plt.show()