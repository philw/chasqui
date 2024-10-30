This is the code I used at the 2024 Autumn Competition in Hazlemere. It's my version of Peter's mazerunner-core.

These switch settings are being used in cli.h

### Setup

#### 9 - test_move()
This moves the mouse forward 100cm. WHEEL_DIAMETER was adjusted to make this as accurate as possible.

#### 10 test_turn()
This turns the mouse in place 360 degrees. MOUSE_RADIUS was adjusted to make this as accurate as possible.

#### 12 see_walls()
This continuously reports which walls the mouse can see. The mouse was placed in different places in the maze to check the sensors.

### Wall Following
Only in place turns are used.

#### 13 slow_wall_follow()
This was my first wall follower code. It stopped in the center of each cell. 

#### 14 less_slow_wall_follow()
In this version the mouse does not stop if it is going straight on.

#### 15 quicker_wall_follow()
This uses the same logic but has higher values for speed and acceleration. This was the version used at Hazlemere.

### Maze Solver

Again, only in place turns are used.

Reset the Arduino with the user button on the main board held down to clear the maze data.

#### 2 search_maze()
This calls the function **search_to(target)**
It searches to the target square updating the map and flooding the maze as it goes.
When it reaches the center it turns around and searches back to the start.

#### 4 run_maze()
This is very like **search_to(target)** but it calls the function **run_to(target)** 
run_to does not update the map or flood the maze. It also uses faster speed and acceleration.

