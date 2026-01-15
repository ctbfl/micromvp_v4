Each car has a controller, it is responsible for generate car actions to move car to accomplish tasks like move somewhere or follow path.

Controller's basic function:
1. Consistently receive `RobotObservation`, and use the information to keep track of it's own status, keep the information in a `CarState`.
2. Keep an statemachine, to handle different tasks and store the state in `CarState.status_label`
3. Provide motion calculation, based on `CarState` and relevant task requirement, output the action to be applied in the environment.
4. Provide exporure API for coordinator to control. This include:
    a. Direct action control (let coordinator overwrite controller's action)
    b. for example, target point setup
    c. for example, target follow traj setup.