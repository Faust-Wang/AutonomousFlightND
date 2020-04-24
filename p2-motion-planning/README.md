## **Project: 3D Motion Planning**

Like in the ```backyardflyer``` project, the ```motion_planning.py``` contains the main code that will provide the commands to guide the drone. 
This program is written using a programming paradigm known as *Event Driven Programming* - this allows the path of the drone to be determined by 
events which occur as opposed to elapsed time, examples of these events include: "Take Off", "Landing", "Arming", etc. 
Due to the threaded nature of the program the drone can still respond to events despite being pre-programmed to follow a path. 
In addition, the program is responsible for calling the planning function so that it can find a path between a starting and a goal location. 
Finding a path is done using a modified version of the ```a_star``` algorithm that it is defined in ```planning_utils.py```.
I have furthered the development of this algorithm by allowing diagonal movements.

### Implementing Your Path Planning Algorithm

#### **Rubric 1:** Set your global home position
The global home position is found from the ```colliders.csv``` file:

```python
with open('colliders.csv', 'r') as fread:
    header = next(fread).strip()

lat_lon = [float(header.split(', ')[i].split(' ')[1]) for i in [0, 1]]
lat0, lon0 = lat_lon[0], lat_lon[1]
```

#### **Rubric 2:** Set your current local position
I set the home position to the latitude and longitude found in the previous step - 
in addition I set the current local position relative to the global position:

```python
self.set_home_position(lon0, lat0, 0)
curr_local_pos = global_to_local(self.global_position, self.global_home)
```

#### **Rubric 3:** Set grid start position from local position
I set the grid start position using:

```python
start = (int(curr_local_pos[0] - north_offset),
         int(curr_local_pos[1] - east_offset))
```

This also accounts for the north/east offsets on the map.

#### **Rubric 4:** Set grid goal position from geodetic coords
The grid goal position is set using:

```python
grid_goal = global_to_local((-122.401247, 37.796738, 0), self.global_home)
grid_goal = (int(grid_goal[0] - north_offset),
             int(grid_goal[1] - east_offset))
```

#### **Rubric 5:** Modify A* to include diagonal motion (or replace A* altogether)
I added four additional actions:

```python
NORTHEAST = (-1, 1, 1.41421)
SOUTHEAST = (1, 1, 1.41421)
SOUTHWEST = (1, -1, 1.41421)
NORTHWEST = (-1, -1, 1.41421)
```

This also required editing of ```valid_actions()``` to ensure the drone could use these actions in the plan:

```python
if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTHEAST)
if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTHEAST)
if (x + 1 > n and y - 1 < 0) or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTHWEST)
if (x - 1 < 0 and y - 1 < 0) or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTHWEST)
```

#### **Rubric 6:** Cull waypoints 
In order to cull the waypoints I use a collinearity check:

```python
def _prune(self, path):

    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(p1, p2, p3, epsilon=0.1):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    p1, p2 = path[0], path[1]
    pruned_path = [p1]
    for i in range(2, len(path)):
        p3 = path[i]
        if collinearity_check(*map(point, [p1, p2, p3])):
            p2 = p3
        else:
            pruned_path.append(p2)
            p1, p2 = p2, p3

    pruned_path.append(p3)
    return pruned_path
```

This examines triplets of points and removes the middle point if the collinearity condition is met.
