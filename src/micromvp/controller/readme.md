Each car has a controller, it is responsible for generate car actions low level)to move car to accomplish tasks like move somewhere or follow path.(high level)

Controller's basic function:
1. Consistently receive `RobotObservation`, and use the information to keep track of it's own status, keep the information in a `CarState`.
2. Keep an statemachine, to handle different tasks and store the state in `CarState.status_label`
3. Provide motion calculation, based on `CarState` and relevant task requirement, output the action to be applied in the environment.
4. Provide exporure API for coordinator to control. For example:
    a. Direct action control (let coordinator overwrite controller's action)
    b. target point setup
    c. target follow traj setup.

For example, a keyboard control framework would looks like:
Code Usage:
```
controllers = {}
for id in ws_config.car_ids:
    controllers[id] = SomeController(robot_id, ws_config, other params) # if you think you need to write a ctrl_config class it is also okay. 

Coordinator = KeyboardControlCoordinator(ws_config, controllers)

observation = env.observe()
while True:
    action = Coordinator.process(observation)
    observation = env.step(action)
    
```

Data Flow:
Observation(a dict contains many car's observation) send into the coordinator, the coordinator would distribute the observations to every controller, to make controller keep track of his own movement.

The coordinator would listen to the keyboard, user could select car use keyboard, and press WASD to control the car. When coordinator receive WASD, it will only send this to the controller, which means the controller should provide a WASD+Stop handle function, and he should be able to resolve the WASD to two wheels speed. Which is simple, just W is (l_speed, r_speed)=(1.0, 1.0), S is (-1, -1), A is (-1, 1), D is (1, -1), and STOP is (0,0).

So inside the KeyboardControlCoordinator, the process function looks like:

```
def process(self, observation):
    # 1. Conduct the high level task
    self.controllers[self.active_car].cmd(self.key_press_map())


    # 2. distribute observation to all cars and get action
    actions = {}
    for ID, obs in observation.items():
        actions[ID] = controllers[ID].step(obs) # update the self-tracking & calculate the action

    return actions
```

Thus inside the controller, it should looks like
```
def step(self, obs):
    # track state
    self.update(obs)
    return self.calculate_action()

def calculation_action():
    if self.car_state.status_label = "W":
        return Action(1,1)
    elif self.carstate.status_label = "S":
        return Action(-1,-1)
    elif ........
    ...

def update(obs: RobotObservation):
    # for information like xy theta, just use the observation data
    # for linear speed and angular speed, need to collect multiple frames to provide an estimate. (need to use 1/ws_config.frequency) as dt.

def cmd(keymap);
    receive keymap, change self.car_state.status_label.

```
NOTE here just and example! in real WASD controller, you should be able to handle multiple key pressed together.