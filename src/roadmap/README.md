# Roadmap
Roadmap node for converting waypoints to polylines. 

Takes hardcoded waypoints and fits lanes through thw waypoints via a Clothoid and/or Cubic spline fitting. The polylines are outputted as `roadmap_msgs/RoadPolylineArray.msg` on `/roadmap/polylines`.

Documentation is found here: [https://lmpcc-roadmap.readthedocs.io/en/latest/index.html](https://lmpcc-roadmap.readthedocs.io/en/latest/index.html)

## Installing
All components should be installed automatically using (to be checked):

`rosdep install --from-paths src --ignore-src -r -y`

The xml reader depends on http://wiki.ros.org/asr_rapidxml.

## Usage
To test the node, run
`roslaunch roadmap roadmap_test.launch`

You should see the following path in RViz.

![example output](https://github.com/oscardegroot/roadmap/blob/main/roadmap/docs/images/test_map_xml.png)

The big cubes denote the input waypoints. All other cubes are fitted waypoints and the splines are visualized by the continuous lines. Each color represents a lane type (e.g., freeway, centerline and crosswalk).

For running the node with other ros nodes, use `roadmap.launch` and see `settings.yaml` for high level settings.
Currenly, the reader supports xml and yaml documents. See `maps/test_map.xml`, `maps/test_map.yaml` for example formats.

**Note:** Support for the yaml files is for backwards compatibility, but the xml format should be prefered.
