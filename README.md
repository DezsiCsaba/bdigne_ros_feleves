# bdigne_ros_feleves
## TurtleSim fraktál(Koch snowflake)
- buildelés & futtatás:
```
cd ros2_ws
colcon build --symlink-install
```

```
ros2 run turtlesim turtlesim_node
ros2 run ros2_course turtlesim_controller
```

- A kód ezt a cikket veszi alapul: https://www.geeksforgeeks.org/koch-curve-koch-snowflake/

## Kód működése
- Anguláris gyorsulás kiszámítása
```python
def get_adjusted_angular_vel(self, angle_error, angular_speed):
  log_comp = math.log((angle_error / 10) + (1 / 20) + 1)
  sin_comp = math.sin(100 * angle_error) * 0.0005
  return (angular_speed * (log_comp + sin_comp))
```
- Az alap loopunk. Rekurzión alapuló függvényhívás
```python
def draw_koch_curve(self, level, size, angular_speed):
  if level == 0:
    self.move_forward(size)
  else:
    self.draw_koch_curve(level - 1, size / 3,angular_speed)
    self.turn_turtle(60,angular_speed)
    self.draw_koch_curve(level - 1, size / 3,angular_speed)
    self.turn_turtle(-120,angular_speed)
    self.draw_koch_curve(level - 1, size / 3,angular_speed)
    self.turn_turtle(60,angular_speed)
    self.draw_koch_curve(level - 1, size / 3,angular_speed)

def main_loop(self, level, size, angular_speed):
  self.move_forward(size/3)
  for _ in range(3):
    self.draw_koch_curve(level, size,angular_speed)
    self.turn_turtle(-120.0,angular_speed)
```

