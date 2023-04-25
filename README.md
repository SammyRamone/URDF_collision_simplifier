What this is
------------
This script automatically simplifies the collision shapes of an URDF.

Typically you export your CAD files and use them to create an URDF. This works well to have nice visual models, but typically you need simpler collision shapes. It is possible to create these manually, but this annoyed me. Therefore, I created this script which automates this process.

The script will automatically choose the best primitive for each link and adapt the URDF file accordingly.


How to use it
-------------

Just run the script as any other Python script. You will need to provide the name of the robot description package and the path to this package, e.g. `python3 simplify_collision.py ROBOT_description ../`

You can chose between different modi.
The default one automatically chooses the best primitive for each link based on the one with the lowest volume.
You can also specify the type yourself or use a convex hull.

Use `python3 simplify_collisions.py --help` to see all options.

Known Limitations
-----------------
I only tested the script on my own xacro based URDF in a ROS 2 package. However, it also should work for other models and ROS 1 packages. Maybe some minor changes are necessary to get it to run. I will happily accept issues and PRs to make this script generalize better on other robots.

Further Information
-------------------

I created it to simplifiy the shapes that I export from SolidWorks using the [exporter plugin](https://github.com/ros/solidworks_urdf_exporter) and [my fork of the sw2urdf_ros2](https://github.com/SammyRamone/sw2urdf_ros2) script which makes a ROS 2 package out of it.

This script uses the very nice [trimesh library](https://trimsh.org/index.html).