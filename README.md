# MazeNavigation
This code was developed by Daniel Álvarez Carreño (TuHechiceroFavorito) and Evanna Niall for the final project of the Robotics Design module of the Engineering course at UCD.

## Purpose
The purpose of the code is to allow the VEXcode's robot (https://vr.vex.com/) to navigate through the dynamic maze map. It will scan all the maze and display in the console a map of how the maze looks like. After it has a value for every wall in the maze, it will solve the maze and navigate from its current position to the end.

## Exploring the maze
The robot will follow the wall that it has on the left. Assuming all walls are conected to the sides of the maze, this strategy guaranties that the robot will navigate to every possible position. It will turn to the left 60º and scan all the walls (or absence of walls) on its way as far as 2 tiles. It was not possible to go further due to imprecisions in the measurements.

With all the information already recorded, the robot will know 'where' is the left wall and will follow it until it has a value for every wall

## Solving the maze
Once it has all the values, it will apply the path finding algorithm, starting on every dead end. The maze has several paths. Just one of those takes to the end. The others take to a dead end. A dead end has 3 walls. It will close these dead ends, setting all walls around the tile. This will make the adjacent open tile a dead end. The same process is applied for this new dead end. This will repeat until it gets to a joint of paths (only 1 wall). After applying this algorithm until there are no more dead ends, the only path left is the one that takes to the end.

## Displaying the map
The robot uses a black square to display a wall and a white square to display the absence of a wall. Each tile is represented by a square made up of 4 squares. The upper ones denote the North position and the left ones the West position. If both positions have a wall (a corner), the tile will be displayed with 3 black squares and 1 white square. With a bit of twicking for a couple of special cases, the robot displays in a neat way all the information recorded during the navigation process.
