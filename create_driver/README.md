# create_driver

## Node

### create_driver

#### Parameters

|         Name         |                    Description                                                              |     Default    |
|----------------------|---------------------------------------------------------------------------------------------|----------------|
| `loop_hz`            | Loop frequency                                                                              | `10`           |
| `dev`                | Device serial port                                                                          | `/dev/ttyUSB0` |
| `latch_cmd_duration` | Time to latch velocity commands before sending zeros                                        | `0.2`          |
| `create_1`           | Boolean flag that should be "true" if using the first generation Create (Roomba 400 series) | `false`        |

#### Launch file

``` bash
$ roslaunch create_driver create.launch [create_1:=true]
```

