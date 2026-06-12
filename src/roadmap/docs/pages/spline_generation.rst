Spline Generation
======================================================
A spline is generated over the received waypoints in two steps.

1. A Clothoid spline is fitted through the waypoints and is sampled to obtain equally spaced waypoints at consistent spacial frequency.
2. Cubic splines are fitted through each segment of the Clothoid spline.


Key Settings
+++++++++++++++++++++++++++++++++++++++++++++++
Step 1 can be skipped by setting ``fit_clothoid`` to ``false`` in ``settings.yaml``. Additional important settings are ``minimum_waypoint_distance``, ``spline_sample_distance``  and ``clothoid_point_per_xm``. They define how the waypoints are spaced throughout the fitting steps and ensure consistent output independent of the input.

The default definition of ``settings.yaml`` is given below

.. code-block:: yaml

	roadmap:
		# === High level settings === #
		debug_output: false   # Print debug information
		update_frequency: 10  # (Hz) Publish map information at this rate
		map_package_name: 'roadmap'         # Package where the map is
		map_file_name: 'maps/test_map.xml'  # Name of the map file
		# map_file_name: "maps/test_map.yaml"
		# map_file_name: 'maps/test_map.osm'

		# === Spline settings === #
		spline:
		fit_clothoid: false             # Fits a clothoid before fitting cubic splines if TRUE
		minimum_waypoint_distance: 2.0  # Ignore waypoints closer than this distance together
		spline_sample_distance: 1       # Distance between sampled waypoints on the spline
		clothoid_point_per_xm: 1        # Distance forced between points on the Clothoid

		# === Visual settings === #
		scale: 1.0                        # Scale of the visuals

		# === Topics === #
		# Topic to read waypoints from INSTEAD of using a predefined map
		external_waypoint_topic: '/carla/ego_vehicle/waypoints'