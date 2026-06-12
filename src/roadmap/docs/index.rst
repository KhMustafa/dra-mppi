.. Roadmap documentation master file, created by
   sphinx-quickstart on Fri Apr 22 17:23:23 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Roadmap
===================================

This package converts input waypoints to output cubic splines that can be used as reference path for a motion planner.

.. figure:: images/test_map_xml.png
   :width: 1000
   :alt: An example output visualized.

   An example output visualized. Large red cubes denote input waypoints. The medium sized cubes show the other defined lanes (two-way road in red and sidewalks in orange). The small cubes and lines denote the fitted spline output.


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   pages/map_specification.rst
   pages/spline_generation.rst
   pages/output.rst
   pages/visualization.rst



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

