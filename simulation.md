# Running Simulations

View this page in GitHub or use VSCode using "ctrl+shift+p" then "Markdown: Open Preview"

## Quick Start

1. Enable desktop simulation
    1. Similar to deploying code, do `ctrl+shift+p`, then enter `Change Desktop Support Enabled Setting`. Set this to true/yes
2. Then `ctrl+shift+p` and enter `Simulate Robot Code`
3. Wait for it to build
4. Check both boxes that appear at the top of VSCode and hit enter
    1. SimGUI will open the simulation GUI interface
    2. Use Real Driver Station will make the local connection to control the simulated robot from the usual Driver Station
5. Adding to the SimGUI
    1. Field visualization: At the top, go to the `NetworkTables > SmartDashboard > Field`
    2. Plots: `Plots > New Plot Window`, then you can drag variables from the NetworkTables window into the plot popup
6. Starting the Sim
    1. The `Robot State` window has the different states you can set the robot to. I usually test in `Autonomous` mode to work with the autos, but `Teleoperated` works too.
    2. For `Teleoperated` to work, a joystick configuration needs to be dragged from the `System Joysticks` into the `Joysticks` window. Think about this as "what's connected to my laptop" versus what is being sent to the simulation.


## How it works

### What is Simulated

* The drivetrain and odometry
    * the angle is simulated by calculated forward the angular rate as a position increase
    * the position is simulated using the drive train kinematics in `SwerveModule` just like usual. The positions for each motor and encoder are simulated by calculating added position from the desired velocities. There is an additional simulation factor (`simulationDriveFactor`) to speed up the robot's response and can be tuned to more accurately model the real robot.
* Shooter, Hood, and Hopper
    * The shooter immediately jumps to the desired RPM
    * The hood immediately jumps to the desired position.

### What is not currently simulated

These are open tasks, and may result in weird/bad interactions with the subsystem. 

* Climber
* Indexer, including hopper
* Intake
* Limelights

### Known Issues

* Constant reporting of the CAN messages being too stale
    * We're not simulating *everything* so some values aren't being updated at the desired rate (if at all).
    * These messages are probably fine as a work in progress
* Loop overruns error messages
    * These report that some periodic function may be running for too long. This is probably okay in sim as long as it's near or below 0.02s but might be cause for concern if we see that when running on the physical robot.

### Nitty Gritty

Fixed assumption that simulation loops occur every 2 milliseconds

Simulated components can be initialized using available libraries from WPILib and Phoenix6 (CANCoder / TalonFX). 

Phoenix6 has simState objects for the encoder and motor that are initialized using the object created for the real device. Function calls on the "real" device instance will be passed along to the simState.

When running in simulation mode, `simulationPeriodic()` is called every 0.020s in the TimedRobot. We've implemented `simulationPeriodic()` functions in the subsystem classes that can be called from the same function in Robot. This handles the periodic updates to the simulated state (such as adding a position travelled based on the goal velocity and the 0.020s time between loops)

To compute some of the required values, additional simulation variables have been created and interspersed in the rest of the code where needed. This should usually be contained inside a logic check confirming the realness of the robot (`Robot.isReal()` or `Robot.isSimulation()`). This conditional allows us to add code that is only executed on the real robot and different code that is only executed in simulation. In general, we should probably try to keep these kinds of checks at a minimum to keep the code cleaner.