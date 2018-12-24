# ca_description

A place for URDF models and meshes for the iRobot Create 2.

## Sources

* Original Create 2 mesh:  https://github.com/goncabrita/roomba_robot


## Debugging tools

### xacro

To convert the xacro file into a URDF file:
```bash
$ roscd ca_description/urdf/
$ roscore &
$ rosrun xacro xacro --inorder create_2.xacro [ARGS] > test.urdf
```
Replace [ARGS] with the corresponding XACRO arguments. As an example, `visualize:=false`.

### URDF

To check whether the sintax is fine or has errors:
```bash
$ check_urdf test.urdf
```

### SDF

To see how Gazebo will use the robot description,
```bash
gzsdf print test.urdf > test.sdf
```
