# behavior_tree_navigation
PS: A lot of the code is taken from Navigation2. This was a project for me to re-build a very simple version of Navigation2 with as little copying as possible.

This repository uses the BehaviorTreeCPP library to add reactive recovery behavior to robot navigation.

Detailed explanation at: https://juhyunglee0313.wixsite.com/portfolio/post/behavior-tree-navigation

# BehaviorTree Navigation
![fig1](https://github.com/Juhyung-L/behavior_tree_navigation/assets/102873080/ff78b729-25f7-4f44-89ad-880f3be13d73)

This behavior tree is implemented. 

The left half of the tree is responsible for navigation. It consists of a recovery node that finds the path from robot's current pose to the goal pose (ComputePathToPose) and another that output a velocity command that follows the path while avoiding obstacles (FollowPath).

The right half of the tree is the recovery behavior. If, for whatever reason, the robot stops moving (maybe it got stuck on an undetected obstacle), the recovery behavior will execute making the robot back up a set distance before trying to move forward again.

[![Watch the video](https://img.youtube.com/vi/qKYPtDzhmn0/maxresdefault.jpg)](https://www.youtube.com/watch?v=qKYPtDzhmn0)
