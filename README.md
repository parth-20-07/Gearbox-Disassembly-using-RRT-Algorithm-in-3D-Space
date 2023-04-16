**RBE550 - Transmission**

<!-- TOC -->

- [Introduction](#introduction)
- [Simulation and Environment](#simulation-and-environment)
- [Implementation](#implementation)
    - [RRT Planner](#rrt-planner)
    - [Collision Checker](#collision-checker)
- [Experimentation](#experimentation)
    - [Restriction based Experimentation](#restriction-based-experimentation)
    - [Search Based Experimentation](#search-based-experimentation)
- [Simulation](#simulation)
- [Results and Graphs](#results-and-graphs)
- [Design Details](#design-details)
- [License](#license)

<!-- /TOC -->

# Introduction
The objective of this task is to disassemble a gearbox with no collisions using RRT Algorithm. We are using the gearbox called SM-465 as shown below

![Gearbox](./Docs/Gearbox%20view.png)

![Gearbox](./Docs/Top%20View.png)

The SM-465 can be found in cars like 1989 Chevrolet Silverado 30.

![1989 Chevvy](https://static1.hotcarsimages.com/wordpress/wp-content/uploads/2022/07/1989-chevrolet-3500_adobe_express.jpeg?q=50&fit=contain&w=1140&h=&dpr=1.5)

A simplified design is provided as shown

![Assembled Box](./Docs/Assembled.png)

Our task is to disassemble the main shaft (top one) as shown and place it on the side.

![Disassembled Box](./Docs/Disassembled.png)

# Simulation and Environment

The complete simulation is done in **Unity Engine *(V2021.3.21f1)*** using C# as shown below with four views

![Simulation Window](./Docs/Image%20Sequence/image_001_0000.jpg)

- Center View: Orthographic View of Enclosed Gearbox design with goal position and motion
- Down Left View: Orthographic View of Enclosed Gearbox Design with goal Position and Motion
- Down Right View: Orthographic View of the RRT Planner sampling nodes where the translucent red shaft is the goal and the translucent orange shaft is the primary shaft trying to sample the environment.
- Top Right View: Right Lid of the Case was Removed which shows the Motion of the primary shaft and the RRT Samples.

![Final Scene Window](./Docs/Image%20Sequence/image_001_10097.jpg)
***Figure: Final Scene***

# Implementation

## RRT Planner

```py
function RRT(start, goal):
    tree = create_tree(start)
    while not reached_goal(tree, goal):
        q_rand = random_point()
        nearest_node = nearest_neighbor(tree, q_rand)
        q_new = steer(nearest_node, q_rand)
        if obstacle_free(nearest_node, q_new):
            add_node(tree, nearest_node, q_new)
            if reached_goal(tree, goal):
                return path(tree, goal)
    return null
```

## Collision Checker

Collision Checking is done using `Physics.ComputePenetration()` command in Unity. The Description of the collision checker is as follows[^1]:
[^1]: Detailed Description at https://docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html

Compute the minimal translation required to separate the given colliders apart at specified poses.

Translating the first collider by direction * distance will separate the colliders apart if the function returned true. Otherwise, direction and distance are not defined.

One of the colliders has to be BoxCollider, SphereCollider CapsuleCollider or a convex MeshCollider. The other one can be any type.

Note that you aren't restricted to the position and rotation the colliders have at the moment of the call. Passing position or rotation that is different from the currently set one doesn't have an effect of physically moving any colliders thus has no side effects on the Scene.

Doesn't depend on any spatial structures to be updated first, so is not bound to be used only within FixedUpdate timeframe.

# Experimentation
## Restriction based Experimentation
- Restricting the Planner to only 2 axes improves the speed to find the path.
- Restricting the domain of the new node search reduces the path planning time.
- Too small of a search domain satisfies the position requirement but fails the orientation requirement for motion.
## Search Based Experimentation
- Setting the node search radius too small makes the path smooth but takes a lot of time.
- Setting the node search radius too big results in the shaft not being able to get out of the Gearbox due to collisions while motioning.

# Simulation

![End Motion](./Docs/Animation.gif)
***Figure: Primary Shaft Motion View***

[![Simulation](https://img.youtube.com/vi/Wp48LX1V3U8/0.jpg)](https://www.youtube.com/watch?v=Wp48LX1V3U8)

***Figure: Complete Simulation View***

# Results and Graphs

![RRT Nodes Generated](./Docs/rrtNodes.png)

***Figure: Nodes Mapped by RRT***

![RRT Path](./Docs/rrtPath.png)

***Figure: RRT Path Generated***

# Design Details

- Designed for:
  - Worcester Polytechnic Institute
  - RBE 550-S23-S01: Motion Planning
- Designed by:
  - [Parth Patel](mailto:parth.pmech@gmail.com)

# License

This project is licensed under [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) (see [LICENSE.md](LICENSE.md)).

Copyright 2023 Parth Patel

Licensed under the GNU General Public License, Version 3.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at

_https://www.gnu.org/licenses/gpl-3.0.en.html_

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.