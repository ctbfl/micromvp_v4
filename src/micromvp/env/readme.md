Each environment should provide it's own `WorkspaceConfig`. (see core/models.py)

It apply action (a dictionary of `Action`) and return observations (A dictionary of `RobotObservation`)

The code to grab observation(e.g. CV code), as well as apply action(e.g. serial sender), should also be put inside the env's folder.