Coordinator is a very flexible element to conduct as a bridge between environment, car controllers, GUI(User), and other high level code logic. 

It will register control panel in GUI.
It will handle the curve and point drawing in GUI.
It will receive the user's input from GUI, and send them to car controllers.
It will hold the main loop, receive environmetn observation from env and distribute them to car controllers, then collect action from car controllers and give the action back to the environment to execute.
It might also need to take care of the collision checking to make sure these cars will not collide into each other.


