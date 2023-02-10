Provides "just enough" infrastructure to spawn `albert_description` into a Gazebo simulation. **Nothing** more.

More complex Gazebo simulations should extend whatever is provided by this package or combine it with other Gazebo simulations. Do **not** change anything here, do **not** make it demo-specific.

Wraps `albert_description` in a new xacro macro which adds the necessary parts (ie: material defs, transmissions, `gazebo_ros_control` plugin, etc).

May also contain `ros_control` configuration specific to `gazebo_ros_control`.

