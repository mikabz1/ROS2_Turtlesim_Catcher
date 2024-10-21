# ROS2 Turtlesim Catcher
## introduction:
This is my first project in ROS2\
The purpose of the project is to practice and implement basic topics in ROS2:\
build packages using colcon.\
- ros2 cli.
- nodes.
- topics
- subscribers.
- Publishers.
- client service.
- service server.
- Custom interfaces.
- Parameters.
- launch files.\

**I implemented the project in both CPP and Python.**

## Project summary:
**Course of the program:**\
Run turtlesim_node and set a turtle to be the central turtle (the robot).\
Then at regular intervals create new turtles.\
The goal of the main turtle (the robot) is to "catch" the other turtles.\
Every time a turtle is caught it is deleted.

**Realization:**\
In order to realize all this I had to create 3 nodes:
- The turtlesim_node from the turtlesim package.
- A custom node to control the turtle (named "turtle1") which is already existing in the turtlesim_node --- controller.
- A custom node to spawn turtles on the window, and to manage which turtle is still "alive" (on the screen) --- spawner.

<ins>The **spawner** node will have to:</ins>
- Call the /spawn service to create a new turtle (choosing random coordinates between 0.0 and 11.0 for both x and y), and call the /kill service to remove a turtle from the screen. Both those services are already advertised by the turtlesim_node.\
- Publish the list of currently alive turtles with coordinates on a topic /alive_turtles.\
- Handling a service server to “catch” a turtle, which means to call the /kill service and remove the turtle from the array of alive turtles.


<ins>The **controller** node will have to:</ins>
- Run a control loop (using a timer with a high rate) to reach a given target point. The first turtle on the screen “turtle1” will be the “master” turtle to control. To control the turtle you can subscribe to /turtle1/pose and publish to /turtle1/cmd_vel.\
- The control loop will use a simplified P controller.\
- Subscribe to the /alive_turtles topic to get all current turtles with coordinates. From that info, select a turtle to target (to catch).\
- When a turtle has been caught by the master turtle, call the service /catch_turtle advertised by the turtle_spawner node.

<ins>custom **interfaces**:</ins>
- Turtle.msg and TurtleArray.msg to send the list of turtles (name + coordinates) on the /alive_turtles topic\
- CatchTurtle.srv to send the name of the turtle which was caught. The client will be the turtle_controller node and the server will be the turtle_spawner node.

![](program run.giff)



