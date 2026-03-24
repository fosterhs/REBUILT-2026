package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.

// Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Shooter shooter = new Shooter(); // Initializes the Shooter subsystem.
  private final Indexer indexer = new Indexer(); // Initializes the Indexer subsystem.
  private final Intake intake = new Intake(); // Initializes the Intake subsystem.

  // Teleop Driving Variables
  private double xVelTeleop = 0.0; // The x-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double yVelTeleop = 0.0; // The y-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double angVelTeleop = 0.0; // The angular velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 
  private final double nearTrenchX = 182.11*0.0254; // The x-position of the near trench on the field in meters. This is used to determine whether the robot is in the near trench.
  private final double farTrenchX = swerve.fieldLength - nearTrenchX; // The x-position of the far trench on the field in meters. This is used to determine whether the robot is in the far trench.
  private final double trenchTolerance = 0.5; // The amount of tolerance that the robot has to be within the trench x-positions to be considered "near" the trench in meters. 
  private boolean isNearTrench = false; // Stores whether the robot is near the trench or not based on its current x-position on the field. 
  private boolean isScoring = false; // Stores whether the robot is currently scoring or passing based on its position on the field. 
  private boolean isShooting = false; // Stores whether the robot is currently shooting or not. Updated in teleop based on driver inputs.
  private final Timer isReadyToShootTimer = new Timer(); // A timer that tracks how long the robot has been ready to shoot. This is used to prevent the indexer from running until the shooter has been up to speed and the robot has been at the shooting position for a certain amount of time, which can help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private final Timer isNotReadyToShootTimer = new Timer(); // A timer that tracks how long the robot has not been ready to shoot. 
  private final double readyOnDelay = 0.3; // The amount of time that the robot needs to be ready to shoot before the isReadyToShoot variable is set to true and allows the indexer to run in seconds. This can help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private final double readyOffDelay = 1.0; // The amount of time that the robot needs to not be ready to shoot before the isReadyToShoot variable is set to false and prevents the indexer from running in seconds. 
  private boolean isReadyToShoot = false; // Stores whether the robot is ready to shoot or not based on whether the shooter has been up to speed and the robot has been at the shooting position for longer than the ready on delay. This variable is used to control whether the indexer should be running or not to help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private boolean isCurrentyReadyToShoot = false; // Stores whether the robot is currently ready to shoot based on whether the shooter is up to speed and the robot is at the shooting position. 
  private boolean currRT = false; // Stores whether the robot is preparing to shoot based on driver inputs. This can be used to start spinning up the shooter and calculating the shooting trajectory before the robot is actually ready to shoot to help improve accuracy and reduce the amount of time it takes for the robot to start shooting once the driver wants to shoot.
  private boolean lastRT = false; // Stores the value of prepareToShoot from the previous iteration of the teleop periodic loop to detect when the driver has just started preparing to shoot.
  private boolean RTPressed = false; // Stores whether the right trigger is currently pressed. This is used to control when the robot is preparing to shoot based on driver inputs.
  private boolean RTReleased = false; 
  private boolean isPreparingToShoot = false;

  // LED Variables
  private final CANBus canivore = new CANBus("canivore"); // Initializes the CANivore CAN Bus for controlling the CANdle.
  private final CANdle topLED = new CANdle(0, canivore); // Initializes the CANdle for controlling the LEDs on the robot. 
  private final SolidColor solidColorRequest = new SolidColor(0, 41); // A SolidColor control request that is used to set the color of the LEDs on the robot. 
  private final RGBWColor purpleColor = new RGBWColor(255, 0, 255, 0); // A purple color for the LEDs to indicate when the robot is not shooting.
  private final RGBWColor greenColor = new RGBWColor(0, 255, 0, 0); // A green color for the LEDs to indicate when the robot is shooting.

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Right pass auto (Manqi)"; 
  private static final String auto2 = "Left Side Start, Neutral Zone then Depot collection. (Bashayer)"; 
  private static final String auto3 = "Center Start, fuel shoot, collect from depot, shoot (Kyle)"; 
  private static final String auto4 = "Troll Auto";
  private static final String auto5 = "Left Side Start, Double Swipe. (Bashayer)";
  private static final String auto6 = "Right Side Start, Double Swipe. (Bashayer)";
  private String autoSelected;
  private int autoStage = 1;
  private boolean autoCompleted = false;
  private final Timer shootingTimer = new Timer();

  // Shooting Trajectory Variables
  private final double hubX = 182.11 * 0.0254; // The x-position of the hub on the field in meters.
  private final double hubY = 158.84 * 0.0254; // The y-position of the hub on the field in meters.
  private final double passingX = 1.5; // The x-position of the point the robot will aim at when passing in meters.
  private final double passingYOffset = 1.5; // The y-distance from the edge of the field that the robot will aim at when passing in meters.
  private double passingY = 1.5; // The y-position of the point the robot will aim at when passing in meters.
  private double robotX; // The current x-position of the robot on the field in meters.
  private double robotY; // The current y-position of the robot on the field in meters.
  private double robotXVel; // The current x-velocity of the robot on the field in meters per second.
  private double robotYVel; // The current y-velocity of the robot on the field in meters per second.
  private double distanceToTarget; // The distance from the robot to the target point in meters.
  private double airTimeApproximation; // An approximation of the time that the fuel will be in the air based on the distance to the target. This is used to calculate how much the robot will need to lead its shots by.
  private double targetX; // The x-position of the point that the robot is aiming at when shooting in meters. This will either be the position of the hub or the position of the passing point, depending on whether the robot is passing or shooting directly at the hub. Includes an offset to lead shots while shooting on the move based on airtime and velocity.
  private double targetY; // The y-position of the point that the robot is aiming at when shooting in meters. This will either be the position of the hub or the position of the passing point, depending on whether the robot is passing or shooting directly at the hub. Includes an offset to lead shots while shooting on the move based on airtime and velocity.
  
  // Sim Variables
  public static final double dTime = 0.020;  // units: seconds
  private final double startingXPosSim = 3.725;  // m
  private final double startingYPosSim = 0.900;  // m

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    autoChooser.addOption(auto5, auto5);
    autoChooser.addOption(auto6, auto6);
    SmartDashboard.putData("Autos", autoChooser);

    // Auto  6 Paths : Double Swipe, Right Starting Position. 0-3
    swerve.loadPath("right, passing pt. 1", 0.0, 0.0, 0.0, 70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("right, passing pt. 2", 0.0, 0.0, 0.0, -167.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("right, passing pt. 3", 0.0, 0.0, 0.0, 70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("right, passing pt. 4", 0.0, 0.0, 0.0, -167.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 2 and 5 Paths : Depot Collection and Double Swipe, Left Starting Position. 4-7
    swerve.loadPath("left, double swipe pt. 1", 0.0, 0.0, 0.0, -70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("left, double swipe pt. 2", 0.0, 0.0, 0.0, 167.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("left, double swipe pt. 3", 0.0, 0.0, 0.0, -70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("left, double swipe pt. 4", 0.0, 0.0, 0.0, 167.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 4 Path : Troll Auto. 
    swerve.loadPath("Troll Auto", 0.0, 0.0, 0.0, 90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 1 passing
    swerve.loadPath("pass auto- first half", 0.0, 0.0, 0.0, 90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("pass auto - second half", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("pass auto- third half.path",0,0,0.0 , 0.0 ); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    shooter.updateDash();
    indexer.updateDash();
    intake.updateDash();
    updateDash();

    if (isReadyToShoot) {
      topLED.setControl(solidColorRequest.withColor(greenColor));
    } else {
      topLED.setControl(solidColorRequest.withColor(purpleColor));
    }
  }

  public void autonomousInit() {
    // Runs the init method for each subsystem to reset them to their default states at the start of autonomous. This is important to ensure that there are no unexpected behaviors caused by leftover states.
    indexer.init();
    intake.init();
    shooter.init();

    swerve.setLimits(1.0, 1.0, 1.0, 1.0);

    // Resets all the shooting variables to their default values at the start of autonomous.
    isCurrentyReadyToShoot = false;
    isReadyToShoot = false;
    isReadyToShootTimer.restart();
    isNotReadyToShootTimer.restart();

    autoCompleted = true;
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        updateTrajectory();
        swerve.resetDriveController(calcHubHeading(3.5, 7.31));
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.pushCalibration(true, -90.0); // Updates the robot's position on the field.
        updateTrajectory();
        swerve.resetDriveController(calcHubHeading(3.5, 7.31));
      break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.pushCalibration(true, -90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(calcHubHeading(2.9, 4.7));
      break;

      case auto4:
        // AutoInit 4 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        updateTrajectory();
        swerve.resetPathController(4); 
      break;

      case auto5:
        // AutoInit 5 code goes here.
        swerve.pushCalibration(true, -90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(calcHubHeading(3.5, 7.31));
      break;

      case auto6:
        // AutoInit 6 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(calcHubHeading(3.5, 0.79));
      break;
    }

    if (Robot.isSimulation()) {
      // Ensure the initial position matches the start of the path
      switch (autoSelected) {
        case auto1:
          swerve.initPathPose(0);
        break;
        case auto2:
          swerve.initPathPose(3);
        break;
        case auto3:
          swerve.initPathPose(6);
        break;
        default:
          swerve.initPathPose(0);
      }
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    isScoring = swerve.getXPos() < nearTrenchX - trenchTolerance; // The robot is considered to be scoring if it's in front of the trench. This is used to determine whether the robot should be aiming for the hub or for the passing point.
    updateTrajectory(); // Updates the shooting trajectory variables based on the current position of the robot on the field. This is used to calculate the optimal shooting parameters for the shooter subsystem.
    shooter.setShootingRPM(calcFlywheelRPM()); // Sets the shooter RPM based on the shooting trajectory calculations.
    indexer.setIndexSpeeds(calcIndexerVoltage()); // Sets the indexer voltage based on the shooting trajectory calculations.

    // Controls the isReadyToShoot variable, which is used to determine whether the indexer should be running or not. The robot needs to be at the shooting position and the shooter needs to be up to speed for a certain amount of time.
    isCurrentyReadyToShoot = shooter.isReady() && swerve.atDriveGoal(); // The robot is currently ready to shoot if the shooter is up to speed and the robot is at the shooting position.
    if (!isCurrentyReadyToShoot) isReadyToShootTimer.restart(); // If the robot is not currently ready to shoot, the timer that tracks how long the robot has been ready to shoot is restarted.
    if (isCurrentyReadyToShoot) isNotReadyToShootTimer.restart(); // If the robot is currently ready to shoot, the timer that tracks how long the robot has not been ready to shoot is restarted.
    if (isReadyToShootTimer.get() > readyOnDelay && !isReadyToShoot) isReadyToShoot = true; // If the robot has been ready to shoot for longer than the ready on delay, the isReadyToShoot variable is set to true, allowing the indexer to run.
    if (isNotReadyToShootTimer.get() > readyOffDelay && isReadyToShoot) isReadyToShoot = false; // If the robot has not been ready to shoot for longer than the ready off delay, the isReadyToShoot variable is set to false, preventing the indexer from running.

    switch (autoSelected) {
     case auto1:
          switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp();
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;


          case 2:
            // Auto 1, Stage 2 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 1.3) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown();//Stops indexers
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(9);
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;


          case 3:
            // Auto 1, Stage 3 code goes here.
            swerve.driveTo(3.519, 0.716,0.0);//drive to this position to make sure path starts correctly
            if (swerve.atDriveGoal()) {
              shooter.spinUp(); //Turns the shooter on
              indexer.spoolUp(); //Start the indexer
              swerve.resetPathController(9); //prepare/reset the path controller to start the path 9
              autoStage = 4; //Advance to the next stage after path controller is resetted
            }
          break;


          case 4:
            // Auto 1, Stage 4 code goes here.
            swerve.followPath(9); //Begin auto path 9
            if (swerve.getXPos() > 5.5) {
              intake.rightIntake(); //Deploy right intake
            }
            if (swerve.getYPos() > 3.0 && swerve.getXPos() > 6.7) { //If robot reached to this coordinate. (near the end of the auto path)
              indexer.stop(); //Stops indexer
              shooter.spinDown(); //Stops the shooter
              indexer.spoolDown(); //Stop indexer
              shooter.lowerHood(); //Stop shooter
              swerve.resetPathController(10); //Prepare the robot to start path 10
              autoStage = 5; //Advance to the next stage after the hood is being lowered, shooter/indexer is stopped
            }
          break;


          case 5:
            // Auto 1, Stage 5 code goes here.
            swerve.followPath(10); //Begin the auto path 10
            if (swerve.getYPos() < 5.8) { //If the robot has passed the trentch
              swerve.resetDriveController(calcHubHeading(3.5, 0.79)); //reset drive controller
              autoStage = 6; //Advance to the next stage after after auto path 10 ended
            }
          break;


          case 6:
            // Auto 1, Stage 6 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); //Drive to this position to start shooting
            shooter.spinUp(); //Start the shooter
            indexer.spoolUp(); //Start the indexer
            shooter.setHoodPosition(calcHoodPosition()); //Adjust hood position
            if (isReadyToShoot) {
              shootingTimer.restart(); //Prepare shooter timer for the next stage
              indexer.start(); //starts the indexer
              autoStage = 7;  //advance to the next stage after it's ready to shoot
            }
          break;


          case 7:
            // Auto 1, Stage 7 code goes here.
            shooter.spinUp(); //make sure shooter continue to spin
            indexer.spoolUp(); //Make sure indexer contine to spin
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); //Keep the robot at this position
            shooter.setHoodPosition(calcHoodPosition()); //Adjust hood shooter
            if (shootingTimer.get() > 2.0) {
              shooter.setHoodPosition(shooter.hoodMinPosition); //Pull the hood back down
              autoStage = 8; //Advance to the next stage after shooting for 2 second
            }
          break;


          case 8:
            // Auto 1, Stage 8 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 1.3) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown();//Stops indexers
              indexer.stop(); // Turns the indexer off.
              autoStage = 9; // Advances to the next stage once the robot has finished shooting.
            }
          break;


          case 9:
            // Auto 1, Stage 9 code goes here.
            swerve.driveTo(3.519, 0.716,0.0);// Drive to this position to make sure path starts correctly
            if (swerve.atDriveGoal()) {
              shooter.spinUp(); // Turns the shooter on
              indexer.spoolUp();// Start the indexer
              swerve.resetPathController(9); // Prepare/reset the path controller to start the path
              autoStage = 10;// Advance to the next stage after path controller has resetted for path 9
            }
          break;


          case 10:
            // Auto 1, Stage 10 code goes here.
            swerve.followPath(9);// Begin the path
            if (swerve.getXPos() > 5.5) {
              intake.rightIntake();// Deploy right intake
            }
            if (swerve.getYPos() > 3.0 && swerve.getXPos() > 6.7) { // If robot reached to this coordinate. (near the end of the auto path)
              indexer.stop(); // Stops indexer
              shooter.spinDown(); // Stops the shooter
              indexer.spoolDown(); // Stop indexer
              shooter.lowerHood(); // Stop shooter
              swerve.resetPathController(10); // Prepare the robot to start the next path
              autoStage = 11;// Advance to the next stage after every thing has stopped and path controll has resetted for path 10
            }
          break;

          case 11:
            // Auto 1, Stage 11 code goes here.
            swerve.followPath(10); // Begin the auto path
            if (swerve.getYPos() < 5.8) { // If the robot has passed the trentch
              swerve.resetDriveController(calcHubHeading(3.5, 0.79)); // Reset drive controller
              autoStage = 12; // Advance to the next stage aftere drive controller is being resetted
            }
          break;


          case 12:
            // Auto 1, Stage 12 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Drive to this position to start shooting
            shooter.spinUp(); // Start the shooter
            indexer.spoolUp();// Start the indexer
            shooter.setHoodPosition(calcHoodPosition()); // Adjust hood position
            if (isReadyToShoot) {
              shootingTimer.restart();// Prepare shooter timer for the next stage
              indexer.start(); // Starts the indexer after it's ready to shoot
              autoStage = 13;  // Advance to the next stage after shooter timer and indexer has restarted
            }
          break;


          case 13:
            // Auto 1, Stage 13 code goes here.
            shooter.spinUp(); // Make sure shooter continue to spin
            indexer.spoolUp(); // Make sure indexer contine to spin
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Keep the robot at this position
            shooter.setHoodPosition(calcHoodPosition()); // Adjust hood shooter
            if (shootingTimer.get() > 2.0) {
              shooter.setHoodPosition(shooter.hoodMinPosition); // Pull the hood back down after two second
            }
          break;
      }
      break;

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp();
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 1.3) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown();
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(4); 
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 3:
            // Auto 2, Stage 3 code goes here.
            swerve.followPath(4); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.0) {
              intake.leftIntake(); // When the X position is greater than 6, the left intake will deploy.
            }
            if (swerve.getYPos() < 4.7) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(5);
              shooter.spinUp();
              indexer.spoolUp();
              autoStage = 4; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 4:
            // Auto 2, Stage 4 code goes here.
            swerve.followPath(5); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              swerve.resetDriveController(calcHubHeading(3.5, 7.31));
              autoStage = 5; // Advances to the next stage once the robot has gotten to a shooting position.
            }
          break;

          case 5:
            // Auto 2, Stage 5 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 6; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 6:
            // Auto 2, Stage 6 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown();
              indexer.stop(); // Turns the indexer off.
              swerve.resetDriveController(0.0);
              autoStage = 8; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 8:
            // Auto 2, Stage 8 code goes here.
            swerve.driveTo(0.5, 7, 0.0); // Brings the robot very backwards.
            if (swerve.getXPos() < 2.0) {
              intake.rightIntake();
            }
            if (swerve.getXPos() < 0.6) {
              swerve.resetDriveController(0.0);
              autoStage = 9; // Advances to the next stage once the robot is ready to intake from the depot.
            }
          break;

          case 9:
            // Auto 2, Stage 9 code goes here.
            swerve.aimDrive(0.0, -2.0, 0.0); // Moves the robot in the depot, collecting fuel.
            if (swerve.getYPos() <= 5.1) {
              intake.stow(); // Stows the intake.
              shooter.spinUp();
              indexer.spoolUp();
              swerve.resetDriveController(calcHubHeading(2.0, 5.1));
              autoStage = 10; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 10:
            // Auto 2, Stage 10 code goes here.
            swerve.driveTo(2.0, 5.1, calcHubHeading(2.0, 5.1)); // Goes to a shooting position.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              indexer.start();
            }
          break;
        }
      break; 

      case auto3:
        switch (autoStage) {
          case 1:
            // Auto 3, Stage 1 code goes here.
            swerve.driveTo(2.9, 4.7, calcHubHeading(2.9, 4.7)); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp();
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (swerve.atDriveGoal()) {
              indexer.start(); // Turns on the indexer.
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 3, Stage 2 code goes here.
            swerve.driveTo(2.0, 4.7, calcHubHeading(2.9, 4.7)); // Brings the robot to a shooting position.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 4.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown();
              indexer.stop(); // Turns the indexer off.
              swerve.resetDriveController(-90.0);
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 3:
            // Auto 3, Stage 3 code goes here.
            swerve.driveTo(1.195, 6.000, -90.0); // Brings the robot to the depot.

            if (swerve.getXPos() < 2.2) {
              intake.rightIntake(); // When the X position is less than 1.8, the right intake will deploy.
            }

            if (swerve.atDriveGoal()) {
              swerve.resetDriveController(-90.0);
              autoStage = 4; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 4:
            // Auto 3, Stage 4 code goes here.
            swerve.aimDrive(-1.0, 0.0, -90.0); // Moves the robot in the depot, collecting fuel.
            if (swerve.getXPos() <= 0.56) {
              intake.stow(); // Stows the intake.
              swerve.resetDriveController(calcHubHeading(2.0, 4.7));
              shooter.spinUp(); // Turns the shooter on.
              indexer.spoolUp();
              autoStage = 5; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 5:
            // Auto 3, Stage 6 code goes here.
            swerve.driveTo(2.0, 4.7, calcHubHeading(2.0, 4.7)); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp();
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot && swerve.atDriveGoal()) {
              indexer.start(); // Turns on the indexer.
            }
          break;
        }
      break; 

      case auto4:
        switch (autoStage) {
          case 1:
            swerve.followPath(8); 
          break;
        }
      break;

      case auto5:
        switch (autoStage) {
          case 1:
            // Auto 5, Stage 1 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp(); // Spins the indexer forward.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 5, Stage 2 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 1.3) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown(); // Stops spinning the indexer.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(4); 
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 3:
            // Auto 5, Stage 3 code goes here.
            swerve.followPath(4); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.0) {
              intake.leftIntake(); // When the X position is greater than 6, the left intake will deploy.
            }
            if (swerve.getYPos() < 4.7) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(5);
              shooter.spinUp(); // Turns the shooter on.
              indexer.spoolUp(); // Spins the indexer forward.
              autoStage = 4; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 4:
            // Auto 5, Stage 4 code goes here.
            swerve.followPath(5); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              swerve.resetDriveController(calcHubHeading(3.5, 7.31));
              autoStage = 5; // Advances to the next stage once the robot has gotten to a shooting position.
            }
          break;

          case 5:
            // Auto 5, Stage 5 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 6; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 6:
            // Auto 5, Stage 6 code goes here.
            swerve.driveTo(3.5, 7.31, calcHubHeading(3.5, 7.31)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown(); // Stops spinning the indexer.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(6); 
              autoStage = 7; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 7:
            // Auto 5, Stage 7 code goes here.
            swerve.followPath(6); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.0) {
              intake.leftIntake(); // When the X position is greater than 6.0, the left intake will deploy.
            }
            if (swerve.getYPos() < 4.7) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(7);
              shooter.spinUp(); // Turns the shooter on.
              indexer.spoolUp(); // Spins the indexer forward.
              autoStage = 8; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 8:
            // Auto 5, Stage 8 code goes here.
            swerve.followPath(7); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShotHeading());
              autoStage = 9; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 9:
            // Auto 5, Stage 9 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShotHeading()); // Rotates the robot to a rotation where it'll have the least misses.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              indexer.start(); // Turns the indexer on.
            }
          break;
        }
      break;

      case auto6:
        switch (autoStage) {
          case 1:
            // Auto 6, Stage 1 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            indexer.spoolUp(); // Spins the indexer forward.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 6, Stage 2 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 1.3) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown(); // Stops spinning the indexer.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(0); 
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 3:
            // Auto 6, Stage 3 code goes here.
            swerve.followPath(0); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.0) {
              intake.rightIntake(); // When the X position is greater than 6, the right intake will deploy.
            }
            if (swerve.getYPos() > 3.4) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(1);
              shooter.spinUp(); // Turns on the shooter.
              indexer.spoolUp(); // Spins the indexer forward.
              autoStage = 4; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 4:
            // Auto 6, Stage 4 code goes here.
            swerve.followPath(1); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              swerve.resetDriveController(calcHubHeading(3.5, 0.79));
              autoStage = 5; // Advances to the next stage once the robot has gotten to a shooting position.
            }
          break;

          case 5:
            // Auto 6, Stage 5 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 6; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 6:
            // Auto 6, Stage 6 code goes here.
            swerve.driveTo(3.5, 0.79, calcHubHeading(3.5, 0.79)); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.spoolDown(); // Stops spinning the indexer.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(6); 
              autoStage = 7; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 7:
            // Auto 6, Stage 7 code goes here.
            swerve.followPath(2); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.0) {
              intake.rightIntake(); // When the X position is greater than 6.0, the left intake will deploy.
            }
            if (swerve.getYPos() < 3.4) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(3);
              shooter.spinUp(); // Turns the shooter on.
              indexer.spoolUp(); // Spins the indexer forward.
              autoStage = 8; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 8:
            // Auto 6, Stage 8 code goes here.
            swerve.followPath(3); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShotHeading());
              autoStage = 9; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 9:
            // Auto 6, Stage 9 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShotHeading()); // Rotates the robot to a rotation where it'll have the least misses.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              indexer.start(); // Turns on the indexer.
            }
          break;
        }
      break;
    }

    // Runs the periodic methods for the subsystems that need to be updated.
    indexer.periodic();
    intake.periodic();
    shooter.periodic();

  }

  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.

    // Initializes the subsystems that need to be initialized for teleop. This is important to reset any variables and states in the subsystems that may have been changed during autonomous.
    indexer.init();
    intake.init();
    shooter.init();

    // Resets all the shooting variables to their default values at the start of teleop.
    lastRT = false;
    currRT = false;
    RTPressed = false;
    RTReleased = false;
    isPreparingToShoot = false;
    isShooting = false;
    isCurrentyReadyToShoot = false;
    isReadyToShoot = false;
    isReadyToShootTimer.restart();
    isNotReadyToShootTimer.restart();
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    isScoring = swerve.getXPos() < nearTrenchX - trenchTolerance; // The robot is considered to be scoring if it's in front of the trench. This is used to determine whether the robot should be aiming for the hub or for the passing point.
    isNearTrench = (nearTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < nearTrenchX + trenchTolerance) || (farTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < farTrenchX + trenchTolerance); // The robot is considered to be near the trench if it's within a certain distance of either edge of the trench. 
    updateTrajectory(); // Updates the optimal shooting trajectory based on the position and velocity of the robot, which is used to calculate the flywheel RPM, hood position, and indexer voltage.
    shooter.setShootingRPM(calcFlywheelRPM()); // Sets the flywheel RPM based on the optimal shooting trajectory.
    indexer.setIndexSpeeds(calcIndexerVoltage()); // Sets the indexer voltage based on the optimal shooting trajectory.

    // Controls the isReadyToShoot variable, which is used to determine whether the indexer should be running or not. The robot needs to be at the shooting position and the shooter needs to be up to speed for a certain amount of time.
    isCurrentyReadyToShoot = shooter.isReady() && swerve.atDriveGoal(); // The robot is currently ready to shoot if the shooter is up to speed and the robot is at the shooting position.
    if (!isCurrentyReadyToShoot) isReadyToShootTimer.restart(); // If the robot is not currently ready to shoot, the timer that tracks how long the robot has been ready to shoot is restarted.
    if (isCurrentyReadyToShoot) isNotReadyToShootTimer.restart(); // If the robot is currently ready to shoot, the timer that tracks how long the robot has not been ready to shoot is restarted.
    if (isReadyToShootTimer.get() > readyOnDelay && !isReadyToShoot) isReadyToShoot = true; // If the robot has been ready to shoot for longer than the ready on delay, the isReadyToShoot variable is set to true, allowing the indexer to run.
    if (isNotReadyToShootTimer.get() > readyOffDelay && isReadyToShoot) isReadyToShoot = false; // If the robot has not been ready to shoot for longer than the ready off delay, the isReadyToShoot variable is set to false, preventing the indexer from running.

    lastRT = currRT;
    if (driver.getRightTriggerAxis() > 0.30) {
      currRT = true;
    } else if (driver.getRightTriggerAxis() < 0.20) {
      currRT = false;
    } else {
      currRT = lastRT;
    }
    RTPressed = currRT && !lastRT;
    RTReleased = !currRT && lastRT;

    // Holding the A button will cause the robot to shoot if it's not in near the trench.
    if (isNearTrench || driver.getRawButtonReleased(1)) {
      isShooting = false; // Releasing the A button or being near the trench will cause the robot to stop shooting.
    } else if (!isNearTrench && driver.getRawButtonPressed(1)) {
      isShooting = true; // Pressing the A button will cause the robot to start shooting if it's not near the trench.
      swerve.resetDriveController(calcShotHeading()); // Resets the drive controller to the current optimal shooting heading to prepare for rotation.
    }

    if (isNearTrench || RTReleased) {
      isPreparingToShoot = false; // Releasing the A button or being near the trench will cause the robot to stop shooting.
    } else if (!isNearTrench && RTPressed) {
      isPreparingToShoot = true; // Pressing the A button will cause the robot to start shooting if it's not near the trench.
      if (!isShooting) {
        swerve.resetDriveController(calcShotHeading()); // Resets the drive controller to the current optimal shooting heading to prepare for rotation.
      }
    }
    

    // The following code allows the driver to toggle between boost mode and default mode with the A and B buttons. In boost mode, the robot will drive at 60% of its maximum speed. In default mode, the robot will drive at 40% of its maximum speed.
    if (driver.getRawButtonPressed(2)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(3)) boostMode = false; // B Button sets default mode (60% of full speed).
    if (isShooting || isPreparingToShoot) {
      swerve.setLimits(0.2, 1.0, 0.4, 1.0);
    } else if (boostMode) {
      swerve.setLimits(1.0, 0.3, 1.0, 1.0);
    } else {
      swerve.setLimits(0.6, 0.3, 1.0, 1.0);
    }

    // The following code controls the swerve lock. If the Y button is pressed, the swerve modules will lock for defense. If any of the joysticks are moved more than 5%, the swerve modules will unlock and the robot will be able to drive again.
    if (driver.getRawButton(4)) { // Y button
      swerveLock = true; // Pressing the Y-button causes the swerve modules to lock (for defense).
      isShooting = false; // The robot cannot be shooting while the swerve is locked.
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    // The following code controls the shooter and indexer. If the robot is in shooting mode, the shooter will spin up and the hood will move to the calculated position. If the shooter is up to speed and the robot is at the shooting position, the indexer will start. If the robot is not in shooting mode, the shooter will spin down, the hood will lower, and the indexer will stop.
    if (isShooting || isPreparingToShoot) {
      shooter.spinUp(); 
      indexer.spoolUp();
      shooter.setHoodPosition(calcHoodPosition());
      if (isReadyToShoot && driver.getRawButton(1)) {
        indexer.start();
      } else {
        indexer.stop();
      }
    } else {
      shooter.spinDown();
      shooter.lowerHood();
      indexer.spoolDown();
      indexer.stop();
    }

    // The following code allows the driver to control the intake with the bumper buttons. If the left bumper is pressed, the intake will deploy and run on the left side. If the right bumper is pressed, the intake will deploy and run on the right side. If either bumper is pressed while that side of the intake is already deployed, the intake will stow.
    if (driver.getLeftBumperButtonPressed()) {
      if (intake.getMode() == Intake.Mode.LEFT) {
        intake.stow();
      } else {
        intake.leftIntake();
      }
    } else if (driver.getRightBumperButtonPressed()) {
      if (intake.getMode() == Intake.Mode.RIGHT) {
        intake.stow();
      } else {
        intake.rightIntake();
      }
    }

    // The following calls are used to update the subsystems and should be called every period.
    indexer.periodic();
    intake.periodic();
    shooter.periodic();

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xDemand = MathUtil.applyDeadband(-driver.getLeftY(), 0.05);
    double yDemand = MathUtil.applyDeadband(-driver.getLeftX(), 0.05);
    double totalDemand = Math.sqrt(Math.pow(xDemand, 2) + Math.pow(yDemand, 2));
    xVelTeleop = xDemand*swerve.maxVelSet;
    yVelTeleop = yDemand*swerve.maxVelSet;
    if (totalDemand > 1.0) {
      xVelTeleop = xVelTeleop/totalDemand;
      yVelTeleop = yVelTeleop/totalDemand;
    }
    angVelTeleop = MathUtil.applyDeadband(-driver.getRightX(), 0.05)*swerve.maxAngVelSet;

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (isShooting || isPreparingToShoot) {
      swerve.aimDrive(xVelTeleop, yVelTeleop, calcShotHeading()); // Allows the driver to adjust the position of the robot while aiming at the hub. The robot will automatically rotate to a rotation where it'll have the least misses.
    } else {
      swerve.drive(xVelTeleop, yVelTeleop, angVelTeleop); // Drive at the velocity demanded by the controller.
    }

    // The following calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the left center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex(); // Calculates which Limelight has the best view of the April Tags and should be used for calibration.
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Parses frames from the Limelight and adds to the calculation of the position of the robot on the field based on visible April Tags.
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Pushes the calculated position of the robot on the field to the drivetrain's odometry once calibration is complete.

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Button 8 is "Menu", the right center button. Sets the current heading of the robot as the new zero. Useful if no April Tags are available, such as driving around the shop.
    
    if (isReadyToShoot) {
      driver.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      driver.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    driver.setRumble(RumbleType.kBothRumble, 0.0);
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    
    autoSelected = autoChooser.getSelected();
    if (!autoCompleted) {
      switch (autoSelected) {
        case auto1:
          swerve.updateVisionHeading(true, 90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto2:
          swerve.updateVisionHeading(true, -90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;
        
        case auto3:
          swerve.updateVisionHeading(true, -90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto4:
          swerve.updateVisionHeading(true, 90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto5:
          swerve.updateVisionHeading(true, -90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto6:
          swerve.updateVisionHeading(true, 90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;
      }
    } else {
      swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    }
    swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), true); 
  }

  public void simulationPeriodic() {
    // Runs at 50 Hz, make sure to call all of the subsystem simulationPeriodic methods
    swerve.simulationPeriodic();
    indexer.simulationPeriodic();
    intake.simulationPeriodic();
    shooter.simulationPeriodic();
  }

  // This method calculates the amount of time the fuel will be in the air based on the distance to the hub and the velocity of the robot. It uses an iterative approach to account for the fact that the aim point changes based on the velocity of the robot and the air time, which changes the distance to the hub, which changes the air time, which changes the aim point, etc. After 10 iterations, the change in air time should be negligible.
  private double[] scoringAirTimeCalibrationDistances = {2.0, 3.0, 4.0, 5.0, 6.0}; // Represents the distance to the hub in meters for each air time calibration value.
  private double[] scoringAirTimeCalibrationValues = {1.1, 1.22, 1.26, 1.46, 1.66}; // Represents the amount of time the fuel will be in the air in seconds for each distance to the hub in the airTimeCalibrationDistances array. The values in this array should correspond to the distances in the airTimeCalibrationDistances array (i.e. the first value in this array is the air time for the first distance in the airTimeCalibrationDistances array, etc.). These values are used to calculate the aim point of the robot based on its velocity and distance to the hub.
  private double[] passingAirTimeCalibrationDistances = {4.0, 6.0, 8.0, 10.0, 12.0}; // Represents the distance to the hub in meters for each air time calibration value.
  private double[] passingAirTimeCalibrationValues = {1.3, 1.6, 1.9, 2.2, 2.5}; // Represents the amount of time the fuel will be in the air in seconds for each distance to the hub in the airTimeCalibrationDistances array. The values in this array should correspond to the distances in the airTimeCalibrationDistances array (i.e. the first value in this array is the air time for the first distance in the airTimeCalibrationDistances array, etc.). These values are used to calculate the aim point of the robot based on its velocity and distance to the hub.
  private void updateTrajectory() {
    robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    robotY = swerve.getYPos(); // The current y-position of the robot on the field in meters.
    robotXVel = swerve.getXVelMeasured(); // The current x-velocity of the robot on the field in meters per second.
    robotYVel = swerve.getYVelMeasured(); // The current y-velocity of the robot on the field in meters per second.

    if (isScoring) {
      distanceToTarget = Math.sqrt(Math.pow(hubX - robotX, 2) + Math.pow(hubY - robotY, 2)); // The distance from the robot to the hub in meters, calculated using the Pythagorean theorem.
      airTimeApproximation = interpolate(distanceToTarget, scoringAirTimeCalibrationDistances, scoringAirTimeCalibrationValues); // The amount of time the fuel will be in the air after being shot, in seconds. Calculated based on distance to the hub using the interpolateAirTime method, which uses a calibration array to return air time values based on distance to the hub.
      targetX = hubX - robotXVel*airTimeApproximation; // The x-position the robot should aim at to account for the movement of the fuel while it's in the air. Calculated by taking the position of the hub and subtracting the distance the fuel will travel while it's in the air (velocity multiplied by airtime).
      targetY = hubY - robotYVel*airTimeApproximation; // The y-position the robot should aim at to account for the movement of the fuel while it's in the air. Calculated by taking the position of the hub and subtracting the distance the fuel will travel while it's in the air (velocity multiplied by airtime).

      for (int i = 0; i < 20; i++) { // Iteratively recalculates the airtime based on the new aim point. This is necessary because the aim point changes the distance the fuel will travel, which changes the airtime, which changes the aim point, etc. After 20 iterations, the change in airtime should be negligible.
        distanceToTarget = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2)); // distance to aim point
        airTimeApproximation = interpolate(distanceToTarget, scoringAirTimeCalibrationDistances, scoringAirTimeCalibrationValues); // Recalculates airtime based on new distance to aim point.
        targetX = hubX - robotXVel*airTimeApproximation; // Recalculates aim point based on new airtime.
        targetY = hubY - robotYVel*airTimeApproximation; // Recalculates aim point based on new airtime.
      }
    } else {
      passingY = robotY > swerve.fieldWidth/2.0 ? swerve.fieldWidth - passingYOffset : passingYOffset;
      distanceToTarget = Math.sqrt(Math.pow(passingX - robotX, 2) + Math.pow(passingY - robotY, 2)); // The distance from the robot to the hub in meters, calculated using the Pythagorean theorem.
      airTimeApproximation = interpolate(distanceToTarget, passingAirTimeCalibrationDistances, passingAirTimeCalibrationValues); // The amount of time the fuel will be in the air after being shot, in seconds. Calculated based on distance to the hub using the interpolateAirTime method, which uses a calibration array to return air time values based on distance to the hub.
      targetX = passingX - robotXVel*airTimeApproximation; // The x-position the robot should aim at to account for the movement of the fuel while it's in the air. Calculated by taking the position of the hub and subtracting the distance the fuel will travel while it's in the air (velocity multiplied by airtime).
      targetY = passingY - robotYVel*airTimeApproximation; // The y-position the robot should aim at to account for the movement of the fuel while it's in the air. Calculated by taking the position of the hub and subtracting the distance the fuel will travel while it's in the air (velocity multiplied by airtime).

      for (int i = 0; i < 20; i++) { // Iteratively recalculates the airtime based on the new aim point. This is necessary because the aim point changes the distance the fuel will travel, which changes the airtime, which changes the aim point, etc. After 20 iterations, the change in airtime should be negligible.
        distanceToTarget = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2)); // distance to aim point
        airTimeApproximation = interpolate(distanceToTarget, passingAirTimeCalibrationDistances, passingAirTimeCalibrationValues); // Recalculates airtime based on new distance to aim point.
        targetX = passingX - robotXVel*airTimeApproximation; // Recalculates aim point based on new airtime.
        targetY = passingY - robotYVel*airTimeApproximation; // Recalculates aim point based on new airtime.
      }
    }
    distanceToTarget = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2)); // distance to aim point
  }

  // This method calculates the RPM the shooter needs to be at to shoot accurately based on the distance to the target. It uses a calibration array to return RPM values based on distance to the target.
  private double[] scoringFlywheelCalibrationDistances = {2.0, 6.0};
  private double[] scoringFlywheelCalibrationValues = {2500.0, 3800.0};
  private double[] passingFlywheelCalibrationDistances = {2.0, 6.0};
  private double[] passingFlywheelCalibrationValues = {2500.0, 3800.0};
  private double calcFlywheelRPM() {
    if (isScoring) {
      return interpolate(distanceToTarget, scoringFlywheelCalibrationDistances, scoringFlywheelCalibrationValues);
    } else {
      return interpolate(distanceToTarget, passingFlywheelCalibrationDistances, passingFlywheelCalibrationValues);
    }
  }

  // This method calculates the position the hood needs to be at to shoot accurately based on the distance to the target. It uses a calibration array to return hood position values based on distance to the target.
  private double[] scoringHoodCalibrationDistances = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0}; 
  private double[] scoringHoodCalibrationValues = {0.02, 0.035, 0.0545, 0.0575, 0.062, 0.065, 0.068, 0.06895, 0.06}; 
  private double[] passingHoodCalibrationDistances = {4.0, 6.0, 8.0, 10.0};
  private double[] passingHoodCalibrationValues = {0.06, 0.066, 0.092, 0.115};
  private double calcHoodPosition() {
    if (isScoring) {
      return interpolate(distanceToTarget, scoringHoodCalibrationDistances, scoringHoodCalibrationValues);
    } else {
      return interpolate(distanceToTarget, passingHoodCalibrationDistances, passingHoodCalibrationValues);
    }
  }

  // This method calculates the voltage the indexer needs to be at to shoot accurately based on the distance to the target. It uses a calibration array to return indexer voltage values based on distance to the target.
  private double[] scoringIndexerCalibrationDistances = {2.0, 6.0};
  private double[] scoringIndexerCalibrationValues = {2666.6, 4000.0};
  private double[] passingIndexerCalibrationDistances = {2.0, 6.0};
  private double[] passingIndexerCalibrationValues = {2666.6, 4000.0};
  private double calcIndexerVoltage() {
    if (isScoring) {
      return interpolate(distanceToTarget, scoringIndexerCalibrationDistances, scoringIndexerCalibrationValues);
    } else {
      return interpolate(distanceToTarget, passingIndexerCalibrationDistances, passingIndexerCalibrationValues);
    }
  }

  // This method calculates the heading the robot needs to be at to shoot accurately based on the position of the robot and the target. It uses basic trigonometry to calculate the angle between the robot and the target, and then adjusts that angle based on which quadrant the target is in relative to the robot.
  private double calcShotHeading() {
    if (robotX < targetX) { // If the target is in front of the robot (i.e. the x position of the target is greater than the x position of the robot), the heading can be calculated directly using the arctangent of the difference in y positions divided by the difference in x positions.
      return Math.toDegrees(Math.atan((targetY - robotY) / (targetX - robotX))); 
    } else if (robotX > targetX) { // If the target is behind the robot (i.e. the x position of the target is less than the x position of the robot), 180 degrees must be added or subtracted from the heading calculated using the arctangent to account for the fact that the robot needs to turn around to face the target. Whether 180 degrees is added or subtracted depends on whether the target is above or below the robot (i.e. whether the y position of the target is greater than or less than the y position of the robot).
      if (robotY <= targetY) {
        return Math.toDegrees(Math.atan((targetY - robotY) / (targetX - robotX))) + 180.0; 
      } else {
        return Math.toDegrees(Math.atan((targetY - robotY) / (targetX - robotX))) - 180.0; 
      }
    } else { // If the robot and the target are in the same vertical line (i.e. they have the same x position), the heading is either 90 or -90 degrees depending on whether the target is above or below the robot. If the target is exactly on top of the robot, the heading doesn't matter, so it returns 0.
      if (robotY > targetY) {
        return -90.0;
      } else if (robotY < targetY) {
        return 90.0;
      } else {
        return 0.0; 
      }
    }
  }

  // Calculates the heading of the hub from a set point on the field assuming a stationary shot. Useful in auto.
  private double calcHubHeading(double xPos, double yPos) {
    if (xPos < hubX) { // If the target is in front of the robot (i.e. the x position of the target is greater than the x position of the robot), the heading can be calculated directly using the arctangent of the difference in y positions divided by the difference in x positions.
      return Math.toDegrees(Math.atan((hubY - yPos) / (hubX - xPos))); 
    } else if (xPos > hubX) { // If the target is behind the robot (i.e. the x position of the target is less than the x position of the robot), 180 degrees must be added or subtracted from the heading calculated using the arctangent to account for the fact that the robot needs to turn around to face the target. Whether 180 degrees is added or subtracted depends on whether the target is above or below the robot (i.e. whether the y position of the target is greater than or less than the y position of the robot).
      if (yPos <= hubY) {
        return Math.toDegrees(Math.atan((hubY - yPos) / (hubX - xPos))) + 180.0; 
      } else {
        return Math.toDegrees(Math.atan((hubY - yPos) / (hubX - xPos))) - 180.0; 
      }
    } else { // If the robot and the target are in the same vertical line (i.e. they have the same x position), the heading is either 90 or -90 degrees depending on whether the target is above or below the robot. If the target is exactly on top of the robot, the heading doesn't matter, so it returns 0.
      if (yPos > hubY) {
        return -90.0;
      } else if (yPos < hubY) {
        return 90.0;
      } else {
        return 0.0; 
      }
    }
  }

  // This is a general interpolation method that takes in an x value and two arrays representing points on a graph and returns an interpolated y value based on the x value. 
  private double interpolate(double x, double[] xArray, double[] yArray) {
    if (xArray.length != yArray.length) { // Checks to make sure the x and y arrays are the same length. If not, it prints an error and returns 0.
      System.out.println("Error: xArray and yArray must be the same length.");
      return 0.0;
    } else if (xArray.length == 0) { // Checks to make sure the x and y arrays have at least one element. If not, it prints an error and returns 0.
      System.out.println("Error: xArray and yArray must have at least one element.");
      return 0.0;
    } else if (x >= xArray[xArray.length - 1]) { // If x is greater than the largest x value in the x array, it returns the corresponding y value for the largest x value (i.e. it assumes the y value stays constant after the largest x value).
      return yArray[yArray.length - 1]; // Return y for largest x
    } else if (x <= xArray[0]) {
      return yArray[0]; // Return y for smallest x
    } else {
      int lowerIndex = -1; // Index for x immediately smaller than current x
      for (int i = 0; i < xArray.length - 1; i++) { 
        if (xArray[i + 1] > x && lowerIndex == -1) {
          lowerIndex = i; // This will be used as the lower point for interpolation.
        }
      } 
      return yArray[lowerIndex] + ((yArray[lowerIndex + 1] - yArray[lowerIndex]) / (xArray[lowerIndex + 1] - xArray[lowerIndex])) * (x - xArray[lowerIndex]);
    }
  }

  // Publishes information to the dashboard.
  private void updateDash() {
    SmartDashboard.putBoolean("Boost Mode", boostMode);
    if (Robot.isSimulation()) {
      SmartDashboard.putNumber("sim/Auto Stage", autoStage);
      SmartDashboard.putBoolean("sim/At Drive Goal", swerve.atDriveGoal());
      SmartDashboard.putBoolean("sim/Shooter Ready", shooter.isReady());

    }
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  private void runAll() { 
    if (Robot.isSimulation()) {
      // Set the robot's initial pose at the beginning of the sim
      swerve.setPoseSim(new Pose2d(startingXPosSim, startingYPosSim, Rotation2d.fromDegrees(0.0)));
    }

    swerve.resetDriveController(0.0);
    swerve.setLimits(1.0, 1.0, 1.0, 1.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.addCalibrationEstimate(0, false);
    swerve.pushCalibration(true, 180.0);
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading(true, 180.0);
    swerve.addVisionEstimate(0, true);
    swerve.updateOdometry();
    swerve.setPosTol(0.20);
    swerve.setAngTol(5.0);
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    swerve.drive(0.01, 0.0, 0.0, true);
    swerve.drive(0.01, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVelMeasured: " + swerve.getAngVelMeasured());
    System.out.println("swerve getXVelDemanded: " + swerve.getXVelDemanded());
    System.out.println("swerve getCalibrationTimer: " + swerve.getVisionTimer());
    System.out.println("swerve getAccurateCalibrationTimer: " + swerve.getAccurateVisionTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getGyroRoll: " + swerve.getGyroRoll());
    System.out.println("swerve getGyroAngVel: " + swerve.getGyroAngVel());
    System.out.println("swerve getGyroPitchVel: " + swerve.getGyroPitchVel());
    System.out.println("swerve getGyroRollVel: " + swerve.getGyroRollVel());
    System.out.println("swerve getPathAngleError: " + swerve.getPathAngleError());
    System.out.println("swerve getPathPosError: " + swerve.getPathPosError());
    System.out.println("swerve getXPos: " + swerve.getXPos());
    System.out.println("swerve getXVelMeasured: " + swerve.getXVelMeasured());
    System.out.println("swerve getXVelDemanded: " + swerve.getXVelDemanded());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVelMeasured: " + swerve.getYVelMeasured());
    System.out.println("swerve getXVelDemadned: " + swerve.getXVelDemanded());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getAngleDist: " + swerve.getAngleDistance(30.0, -120.0));
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    swerve.updateDash();

    shooter.init();
    shooter.periodic();
    shooter.spinUp();
    shooter.spinDown();
    shooter.setShootingRPM(3000.0);
    shooter.setHoodPosition(calcHoodPosition());
    shooter.lowerHood();
    System.out.println("shooter hoodIsInPosition: " + shooter.hoodIsInPosition());
    System.out.println("shooter getHoodPosition: " + shooter.getHoodPosition());
    System.out.println("shooter flywheelIsAtSpeed(): " + shooter.flywheelIsAtSpeed());    
    System.out.println("shooter getLeftShooterRPM: " + shooter.getLeftFlywheelMotorRPM());
    System.out.println("shooter getRightShooterRPM: " + shooter.getRightFlywheelMotorRPM());
    System.out.println("shooter isReady: " + shooter.isReady());
    System.out.println("shooter getMode: " + shooter.getMode().toString());
    shooter.updateDash();

    indexer.init();
    indexer.periodic();
    indexer.start();
    indexer.stop();
    indexer.spoolUp();
    indexer.spoolDown();
    indexer.setIndexSpeeds(12.0);
    System.out.println("indexer getMode: " + indexer.getMode().toString());
    indexer.updateDash();

    intake.init();
    intake.periodic();
    intake.leftIntake();
    intake.rightIntake();
    intake.stow();
    System.out.println("intake getMode: " + intake.getMode().toString());
    System.out.println("intake getLeftArmPosition: " + intake.getLeftArmPosition());
    System.out.println("intake getLeftArmDesiredPosition: " + intake.getLeftArmDesiredPosition());
    System.out.println("intake leftArmInPosition: " + intake.leftArmInPosition());
    System.out.println("intake getRightArmPosition: " + intake.getRightArmPosition());
    System.out.println("intake getRightArmDesiredPosition: " + intake.getRightArmDesiredPosition());
    System.out.println("intake rightArmInPosition: " + intake.rightArmInPosition());
    System.out.println("intake isReady: " + intake.isReady());
    intake.updateDash();

    updateTrajectory();
    System.out.println("calcShooterRPM: " + calcFlywheelRPM());
    System.out.println("calcHoodPosition: " + calcHoodPosition());
    System.out.println("getHubHeading: " + calcShotHeading());
    updateDash();
  }
}
