This is a trajectory saver packagefor use with ROS and the anscer AR100 simulation the `trajectory_saver` node lisntes to the tf messages and publishes the odom to base
-link transform as a Marker Array, it also offers a service called `SaveRequest` that takes a `filename` and `durartion` as arguments,
it writes data from the Marker Array for the last `duration` seconds into the `flename` file in a CSV format. The `trajectory_reader` node takes the `filename` as an argument, and reads it and parses `filename` as a CSV file and publishes the data as a marker array.
