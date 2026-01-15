# GUI.

The GUI is basically a left control panel + a canvas on the right.

The GUI will first be initialize with information of the `WorkspaceConfig`.

## Car related most basic functions:

On the buttom of the left control panel, there should be a car info inspecter. When user click on a car on the right, the corresponding information would be displayed. This include all term inside `CarState` except for matadata.

On the right canvas, it should display the workspace using a square. The canvas size can be drag to adjust by user. It should always try to fit the canvas space. But only scale it, no reshape. Also the displayed car_img should be resized accordingly.
It means GUI need to keep a mapping factor/transformation between input carstate and the screen pixels.

In the workspace, the canvas should show every car according to the given list of `CarState`, precisely demonstrate it's location and orientation.  And there should be a Car ID (number only) near the car( e.g. to the top left of the car)

# Customized Left Control Panel setup

The left control panel can be easily registered via API. (Usually setup by coordinator)
When user change those value, the corresponding callback function will be called.

The left panel should support:
1. toggles
2. slider with no level (continuous)
3. slider with level (discrete, can set each level)

I hope this can be accomplished by a simple json like format. All controlable items arrange from top towards bottom according to the sequence.

# Customized Right Canvas

The right canvas need also provides some interactive API to get user interaction, and some active API to demonstrate more system information.

(type 1: user interaction API)
1. Draw Point callback: user draw a point on canvas, the position of the point would be sent to registered callback function.
2. Draw curves callback: user draw a curve on canvas, the whole curve will be sent to registered callback function.
3. Click car callback: user click on a car on canvas, the registered callback function should be called.

(type 2: system info demonstration)
3. Active Draw Point: Actively draw a point on canvas with specific color/size, return a point element that can be called to be destroyed. (Use workspace unit)
4. Active Draw Curve: Actively draw a curve on canvas with specific color/boldness, return a curve element that can be called to be destroyed. (Use workspace unit)
5. Active Draw Text: Actively draw a curve on canvas with specific font/size/color, return a text element that can be called to be destroyed. (Use workspace unit)