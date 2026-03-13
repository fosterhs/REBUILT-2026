package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  // Teleop Driving Variables
  private double xVelTeleop = 0.0; // The x-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double yVelTeleop = 0.0; // The y-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double angVelTeleop = 0.0; // The angular velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double speedScaleFactor = 0.4; // Scales the translational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private double rotationScaleFactor = 0.3; // Scales the rotational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 
  private final double nearTrenchX = 182.11*0.0254; // The x-position of the near trench on the field in meters. This is used to determine whether the robot is in the near trench.
  private final double farTrenchX = Drivetrain.fieldLength - nearTrenchX; // The x-position of the far trench on the field in meters. This is used to determine whether the robot is in the far trench.
  private final double trenchTolerance = 0.5; // The amount of tolerance that the robot has to be within the trench x-positions to be considered "near" the trench in meters. 
  private boolean isNearTrench = false; // Stores whether the robot is near the trench or not based on its current x-position on the field. 
  private boolean isScoring = false; // Stores whether the robot is currently scoring or passing based on its position on the field. 
  private boolean isShooting = false; // Stores whether the robot is currently shooting or not. Updated in teleop based on driver inputs.
  private final Timer isReadyToShootTimer = new Timer(); // A timer that tracks how long the robot has been ready to shoot. This is used to prevent the indexer from running until the shooter has been up to speed and the robot has been at the shooting position for a certain amount of time, which can help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private final Timer isNotReadyToShootTimer = new Timer(); // A timer that tracks how long the robot has not been ready to shoot. 
  private final double readyOnDelay = 0.3; // The amount of time that the robot needs to be ready to shoot before the isReadyToShoot variable is set to true and allows the indexer to run in seconds. This can help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private final double readyOffDelay = 2.0; // The amount of time that the robot needs to not be ready to shoot before the isReadyToShoot variable is set to false and prevents the indexer from running in seconds. 
  private boolean isReadyToShoot = false; // Stores whether the robot is ready to shoot or not based on whether the shooter has been up to speed and the robot has been at the shooting position for longer than the ready on delay. This variable is used to control whether the indexer should be running or not to help improve accuracy by ensuring that fuel is not fed into the shooter until it's ready.
  private boolean isCurrentyReadyToShoot = false; // Stores whether the robot is currently ready to shoot based on whether the shooter is up to speed and the robot is at the shooting position. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  //private final Climber climber = new Climber(); // Initializes the Climber subsystem.
  private final Shooter shooter = new Shooter(); // Initializes the Shooter subsystem.
  private final Indexer indexer = new Indexer(); // Initializes the Indexer subsystem.
  private final Intake intake = new Intake(); // Initializes the Intake subsystem.

  // LED Variables
  private final CANBus canivore = new CANBus("canivore"); // Initializes the CANivore CAN Bus for controlling the CANdle.
  private final CANdle topLED = new CANdle(0, canivore); // Initializes the CANdle for controlling the LEDs on the robot. 
  private final SolidColor solidColorRequest = new SolidColor(8, 41); // A SolidColor control request that is used to set the color of the LEDs on the robot. 
  private final RGBWColor purpleColor = new RGBWColor(255, 0, 255, 0); // A purple color for the LEDs to indicate when the robot is not shooting.
  private final RGBWColor greenColor = new RGBWColor(0, 255, 0, 0); // A green color for the LEDs to indicate when the robot is shooting.

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Fuel Collection Via Neutral Zone, Right Side Start."; 
  private static final String auto2 = "Fuel Collection Via Neutral Zone, Left Side Start."; 
  private static final String auto3 = "Climbing, Center Start."; 
  private static final String auto4 = "Right Side. Shoot, collect from neutral zone, shoot, collect fuel from the human player station."; 
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
    SmartDashboard.putData("Autos", autoChooser);

    // Auto 1 Paths : Fuel Collection from Neutral Zone, Right Starting Position. 0-1
    swerve.loadPath("neutral zone right travelling to zone", 0.0, 0.0, 0.0, 70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone right travelling to shooting position 2", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 2 Paths : Fuel Collection from Neutral Zone, Left Starting Position. 2-3
    swerve.loadPath("neutral zone left travelling to zone", 0.0, 0.0, 0.0, -70.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone left travelling to shooting position 2", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 3 Paths : Climbing Auto, Center Starting Position. 4-6
    swerve.loadPath("climbing travelling to depot", 0.0, 0.0, 0.0, -15.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("climbing travelling to shooting position 2", 0.0, 0.0, 0.0, 90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("climbing travelling to tower", 0.0, 0.0, 0.0, -15.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    //climber.updateDash();
    shooter.updateDash();
    indexer.updateDash();
    intake.updateDash();
    updateDash();

    if (isShooting) {
      topLED.setControl(solidColorRequest.withColor(greenColor));
    } else {
      topLED.setControl(solidColorRequest.withColor(purpleColor));
    }
  }

  public void autonomousInit() {
    // Runs the init method for each subsystem to reset them to their default states at the start of autonomous. This is important to ensure that there are no unexpected behaviors caused by leftover states.
    //climber.init(); 
    indexer.init();
    intake.init();
    shooter.init();

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
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.pushCalibration(true, -90.0); // Updates the robot's position on the field.
      break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.pushCalibration(true, 180.0); // Updates the robot's position on the field.
        swerve.resetPathController(4); 
      break;

      case auto4:
        // AutoInit 4 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(calcShootingHeading());
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

    isScoring = true; // The robot is scoring during the entire autonomous period, so this variable is set to true to allow the shooting trajectory to update and be used in the shooter calculations.
    updateTrajectory(); // Updates the shooting trajectory variables based on the current position of the robot on the field. This is used to calculate the optimal shooting parameters for the shooter subsystem.
    shooter.setShootingRPM(calcFlywheelRPM()); // Sets the shooter RPM based on the shooting trajectory calculations.
    indexer.setIndexVoltage(calcIndexerVoltage()); // Sets the indexer voltage based on the shooting trajectory calculations.

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
            swerve.drive(-0.3, 0.0, 0.0, true, 0.0, 0.0); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.getXPos() <= 3.5) {
              swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot.
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShootingHeading()); // Rotates the robot to a rotation where it'll have the least misses.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 1, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter. 
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(0); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 1, Stage 4 code goes here.
            swerve.followPath(0); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 5.5) {
                intake.rightIntake(); // When the X position is greater than 5.5, the right intake will deploy.
            }
            if (swerve.atPathEndpoint(0) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 1, Stage 5 code goes here.
            swerve.driveTo(7.700,3.575,-90.719); // Moves the robot in the neutral zone, collecting fuel.
            if (swerve.atDriveGoal()) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(1);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 1, Stage 6 code goes here.
            swerve.followPath(1); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(1)) {
              swerve.resetDriveController(calcShootingHeading());
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 1, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 1, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (shootingTimer.get() > 2.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.stop(); // Turns the indexer off.
            }
          break;
        }
      break; 

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.driveTo(3.500,7.310,-177.723); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atDriveGoal()) {
              swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot.
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShootingHeading());
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 2, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (shootingTimer.get() > 2.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(2); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 2, Stage 4 code goes here.
            swerve.followPath(2); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 5.5) {
                intake.leftIntake(); // When the X position is greater than 5.5, the left intake will deploy.
            }
            if (swerve.atPathEndpoint(2) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 2, Stage 5 code goes here.
            swerve.driveTo(7.700,4.475, 89.098);// Moves the robot in the neutral zone, collecting fuel.
            if (swerve.atDriveGoal()) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(3);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 2, Stage 6 code goes here.
            swerve.followPath(3); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(3)) {
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShootingHeading());
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 2, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 2, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
            }
          break;
        }
      break; 

      case auto3:
        switch (autoStage) {
          case 1:
            // Auto 3, Stage 1 code goes here.
            swerve.driveTo(3.51, 5.1, calcShootingHeading()); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(4)) {
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShootingHeading());
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 3, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 3, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(4); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 3, Stage 4 code goes here.
            swerve.followPath(4); // Brings the robot to the depot.
            if (swerve.getXPos() < 1.8) {
                intake.leftIntake(); // When the X position is less than 1.8, the left intake will deploy.
            }
            if (swerve.atPathEndpoint(4) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 3, Stage 5 code goes here.
            swerve.drive(0.0, 1.0, 0.0, true, 0.0, 0.0); // Moves the robot in the depot, collecting fuel.
            if (swerve.getYPos() >= 6.1) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(5);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 3, Stage 6 code goes here.
            swerve.followPath(5); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(5)) {
              shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
              swerve.resetDriveController(calcShootingHeading());
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 3, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, calcShootingHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              shootingTimer.restart(); // Restarts the shooting timer.
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 3, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.stop(); // Turns the indexer off.
              //climber.moveUp(); // Moves the climber up.
              swerve.resetPathController(6);
              autoStage = 9; // Moves onto the next stage once the robot has finished shooting.
            }
          break;

          case 9:
            // Auto 3, Stage 9 code goes here.
            swerve.followPath(6); // Brings the robot to the tower to climb.
            if (swerve.atPathEndpoint(6)) {
              //climber.moveDown(); // Moves the climber down so the robot is actually climbing the rung.
            }
          break;
        }
        break;

      case auto4:
        switch (autoStage) {
          case 1:
            // Auto 4, Stage 1 code goes here.
            swerve.driveTo(3.5, 0.75, calcShootingHeading()); // Brings the robot slightly backwards.
            shooter.spinUp(); // Turns the shooter on.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              shootingTimer.restart(); // Restarts the shooting timer.
              indexer.start(); // Turns on the indexer.
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 4, Stage 2 code goes here.
            swerve.driveTo(3.5, 0.75, calcShootingHeading()); // Brings the robot slightly backwards.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (shootingTimer.get() > 4.0) {
              shooter.spinDown(); // Turns the shooter off.
              shooter.lowerHood(); // Lowers the hood of the shooter.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(0); 
              autoStage = 3; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 3:
            // Auto 4, Stage 3 code goes here.
            swerve.followPath(0); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 5.5) {
              intake.rightIntake(); // When the X position is greater than 5.5, the right intake will deploy.
            }
            if (swerve.getXPos() > 7.3) {
              autoStage = 4; // Advances to the next stage once the robot has gotten to the neutral zone.
              swerve.resetDriveController(180.0);
            }
          break;

          case 4:
            // Auto 4, Stage 4 code goes here.
            swerve.aimDrive(0.0, 2.0, 180.0, true); // Moves the robot in the neutral zone, collecting fuel.
            if (swerve.getYPos() > 3.2) {
              intake.stow(); // Stows the intake.
              swerve.resetPathController(1);
              shooter.spinUp(); // Turns the shooter on.
              autoStage = 5; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 5:
            // Auto 4, Stage 5 code goes here.
            swerve.followPath(1); // Brings the robot back to a shooting position from the neutral zone.
            if (swerve.getXPos() < 3.75) {
              swerve.resetDriveController(calcShootingHeading());
              autoStage = 6; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 6:
            // Auto 4, Stage 6 code goes here.
            swerve.driveTo(0.61, 0.65, calcShootingHeading()); // Brings the robot to the outpost for fuel.
            shooter.setHoodPosition(calcHoodPosition()); // Sets the hood position to shoot as accurately as possible.
            if (isReadyToShoot) {
              indexer.start(); // Turns on the indexer.
            }
          break; 
        }
      break;
    }

    // Runs the periodic methods for the subsystems that need to be updated.
    //climber.periodic();
    indexer.periodic();
    intake.periodic();
    shooter.periodic();
  }

  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.

    // Initializes the subsystems that need to be initialized for teleop. This is important to reset any variables and states in the subsystems that may have been changed during autonomous.
    //climber.init(); 
    indexer.init();
    intake.init();
    shooter.init();

    // Resets all the shooting variables to their default values at the start of teleop.
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
    indexer.setIndexVoltage(calcIndexerVoltage()); // Sets the indexer voltage based on the optimal shooting trajectory.

    // Controls the isReadyToShoot variable, which is used to determine whether the indexer should be running or not. The robot needs to be at the shooting position and the shooter needs to be up to speed for a certain amount of time.
    isCurrentyReadyToShoot = shooter.isReady() && swerve.atDriveGoal(); // The robot is currently ready to shoot if the shooter is up to speed and the robot is at the shooting position.
    if (!isCurrentyReadyToShoot) isReadyToShootTimer.restart(); // If the robot is not currently ready to shoot, the timer that tracks how long the robot has been ready to shoot is restarted.
    if (isCurrentyReadyToShoot) isNotReadyToShootTimer.restart(); // If the robot is currently ready to shoot, the timer that tracks how long the robot has not been ready to shoot is restarted.
    if (isReadyToShootTimer.get() > readyOnDelay && !isReadyToShoot) isReadyToShoot = true; // If the robot has been ready to shoot for longer than the ready on delay, the isReadyToShoot variable is set to true, allowing the indexer to run.
    if (isNotReadyToShootTimer.get() > readyOffDelay && isReadyToShoot) isReadyToShoot = false; // If the robot has not been ready to shoot for longer than the ready off delay, the isReadyToShoot variable is set to false, preventing the indexer from running.

    // Holding the A button will cause the robot to shoot if it's not in near the trench.
    if (isNearTrench || driver.getRawButtonReleased(1)) {
      isShooting = false; // Releasing the A button or being near the trench will cause the robot to stop shooting.
    } else if (!isNearTrench && driver.getRawButtonPressed(1)) {
      isShooting = true; // Pressing the A button will cause the robot to start shooting if it's not near the trench.
      swerve.resetDriveController(calcShootingHeading()); // Resets the drive controller to the current optimal shooting heading to prepare for rotation.
    }

    // The following code allows the driver to toggle between boost mode and default mode with the A and B buttons. In boost mode, the robot will drive at 60% of its maximum speed. In default mode, the robot will drive at 40% of its maximum speed.
    if (driver.getRawButtonPressed(2)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(3)) boostMode = false; // B Button sets default mode (60% of full speed).
    speedScaleFactor = boostMode ? 1.0 : 0.6; // If boost mode is enabled, the speed scale factor is 1.0, otherwise it's 0.6.

    // The following code controls the swerve lock. If the Y button is pressed, the swerve modules will lock for defense. If any of the joysticks are moved more than 5%, the swerve modules will unlock and the robot will be able to drive again.
    if (driver.getRawButton(4)) { // Y button
      swerveLock = true; // Pressing the Y-button causes the swerve modules to lock (for defense).
      isShooting = false; // The robot cannot be shooting while the swerve is locked.
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    // The following code controls the shooter and indexer. If the robot is in shooting mode, the shooter will spin up and the hood will move to the calculated position. If the shooter is up to speed and the robot is at the shooting position, the indexer will start. If the robot is not in shooting mode, the shooter will spin down, the hood will lower, and the indexer will stop.
    if (isShooting) {
      shooter.spinUp(); 
      shooter.setHoodPosition(calcHoodPosition());
      if (isReadyToShoot) {
        indexer.start();
      } else {
        indexer.stop();
      }
    } else {
      shooter.spinDown();
      shooter.lowerHood();
      indexer.stop();
    }

    // The following code allows the driver to control the climber with the trigger buttons. If the right trigger is pressed, the climber will move up. If the left trigger is pressed, the climber will move down. If the robot is past a certain point on the field, the climber will stow automatically.
    if (swerve.getXPos() > 2.0) {
      //climber.stow();
    } else if (driver.getRightTriggerAxis() > 0.25) {
      //climber.moveUp();
    } else if (driver.getLeftTriggerAxis() > 0.25) {
      //climber.moveDown(); 
    } 

    if (driver.getPOV() == 0) intake.home(); // D-pad up homes the intake incase the robot needs to recalibrate the position of the intake.

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
    //climber.periodic(); 
    indexer.periodic();
    intake.periodic();
    shooter.periodic();

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    xVelTeleop = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    yVelTeleop = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    angVelTeleop = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*rotationScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (isShooting) {
      swerve.aimDrive(xVelTeleop, yVelTeleop, calcShootingHeading(), true); // Allows the driver to adjust the position of the robot while aiming at the hub. The robot will automatically rotate to a rotation where it'll have the least misses.
    } else {
      swerve.drive(xVelTeleop, yVelTeleop, angVelTeleop, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the left center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex(); // Calculates which Limelight has the best view of the April Tags and should be used for calibration.
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Parses frames from the Limelight and adds to the calculation of the position of the robot on the field based on visible April Tags.
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Pushes the calculated position of the robot on the field to the drivetrain's odometry once calibration is complete.

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Button 8 is "Menu", the right center button. Sets the current heading of the robot as the new zero. Useful if no April Tags are available, such as driving around the shop.
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
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
          swerve.updateVisionHeading(true, 180.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto4:
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
    //climber.simulationPeriodic();
    indexer.simulationPeriodic();
    intake.simulationPeriodic();
    shooter.simulationPeriodic();
  }

  // This method calculates the amount of time the fuel will be in the air based on the distance to the hub and the velocity of the robot. It uses an iterative approach to account for the fact that the aim point changes based on the velocity of the robot and the air time, which changes the distance to the hub, which changes the air time, which changes the aim point, etc. After 10 iterations, the change in air time should be negligible.
  private double[] scoringAirTimeCalibrationDistances = {2.0, 3.0, 4.0, 5.0, 6.0}; // Represents the distance to the hub in meters for each air time calibration value.
  private double[] scoringAirTimeCalibrationValues = {1.1, 1.22, 1.26, 1.46, 1.66}; // Represents the amount of time the fuel will be in the air in seconds for each distance to the hub in the airTimeCalibrationDistances array. The values in this array should correspond to the distances in the airTimeCalibrationDistances array (i.e. the first value in this array is the air time for the first distance in the airTimeCalibrationDistances array, etc.). These values are used to calculate the aim point of the robot based on its velocity and distance to the hub.
  private double[] passingAirTimeCalibrationDistances = {2.0, 3.0, 4.0, 5.0, 6.0}; // Represents the distance to the hub in meters for each air time calibration value.
  private double[] passingAirTimeCalibrationValues = {0.6, 0.7, 0.8, 0.9, 1.0}; // Represents the amount of time the fuel will be in the air in seconds for each distance to the hub in the airTimeCalibrationDistances array. The values in this array should correspond to the distances in the airTimeCalibrationDistances array (i.e. the first value in this array is the air time for the first distance in the airTimeCalibrationDistances array, etc.). These values are used to calculate the aim point of the robot based on its velocity and distance to the hub.
  private void updateTrajectory() {
    robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    robotY = swerve.getYPos(); // The current y-position of the robot on the field in meters.
    robotXVel = swerve.getXVel(); // The current x-velocity of the robot on the field in meters per second.
    robotYVel = swerve.getYVel(); // The current y-velocity of the robot on the field in meters per second.

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
      passingY = robotY > Drivetrain.fieldWidth/2.0 ? Drivetrain.fieldWidth - passingYOffset : passingYOffset;
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
  private double[] scoringHoodCalibrationDistances = {2.0, 3.0, 4.0, 5.0, 6.0}; 
  private double[] scoringHoodCalibrationValues = {0.0515, 0.058, 0.0625, 0.065, 0.06}; 
  private double[] passingHoodCalibrationDistances = {2.0, 3.0, 4.0, 5.0, 6.0};
  private double[] passingHoodCalibrationValues = {0.05, 0.0625, 0.07, 0.065, 0.06};
  private double calcHoodPosition() {
    if (isScoring) {
      return interpolate(distanceToTarget, scoringHoodCalibrationDistances, scoringHoodCalibrationValues);
    } else {
      return interpolate(distanceToTarget, passingHoodCalibrationDistances, passingHoodCalibrationValues);
    }
  }

  // This method calculates the voltage the indexer needs to be at to shoot accurately based on the distance to the target. It uses a calibration array to return indexer voltage values based on distance to the target.
  private double[] scoringIndexerCalibrationDistances = {2.0, 6.0};
  private double[] scoringIndexerCalibrationValues = {8.0, 12.0};
  private double[] passingIndexerCalibrationDistances = {2.0, 6.0};
  private double[] passingIndexerCalibrationValues = {8.0, 12.0};
  private double calcIndexerVoltage() {
    if (isScoring) {
      return interpolate(distanceToTarget, scoringIndexerCalibrationDistances, scoringIndexerCalibrationValues);
    } else {
      return interpolate(distanceToTarget, passingIndexerCalibrationDistances, passingIndexerCalibrationValues);
    }
  }

  // This method calculates the heading the robot needs to be at to shoot accurately based on the position of the robot and the target. It uses basic trigonometry to calculate the angle between the robot and the target, and then adjusts that angle based on which quadrant the target is in relative to the robot.
  private double calcShootingHeading() {
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
    SmartDashboard.putNumber("ToHubDis", distanceToTarget);
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
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
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
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
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getAccurateCalibrationTimer: " + swerve.getAccurateCalibrationTimer());
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
    System.out.println("swerve getXVel: " + swerve.getXVel());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVel: " + swerve.getYVel());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getAngleDist: " + swerve.getAngleDistance(30.0, -120.0));
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    swerve.updateDash();

    //climber.init();
    //climber.periodic();
    //climber.moveUp();
    //climber.moveDown();
    //climber.stow();
    //System.out.println("climber getMode: " + climber.getMode().toString());
    //System.out.println("climber atDesiredPosition: " + climber.atDesiredPosition());
    //System.out.println("climber getPosition: " + climber.getPosition());
    //System.out.println("climber getVelocity: " + climber.getVelocity());
    //climber.updateDash();

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
    indexer.setIndexVoltage(12.0);
    System.out.println("indexer getMode: " + indexer.getMode().toString());
    indexer.updateDash();

    intake.init();
    intake.periodic();
    intake.leftIntake();
    intake.rightIntake();
    intake.stow();
    System.out.println("intake getMode: " + intake.getMode().toString());
    System.out.println("intake getLeftArmPosition: " + intake.getLeftArmPosition());
    System.out.println("intake getLeftArmVelocity: " + intake.getLeftArmVelocity());
    System.out.println("intake getLeftArmDesiredPosition: " + intake.getLeftArmDesiredPosition());
    System.out.println("intake leftArmInPosition: " + intake.leftArmInPosition());
    System.out.println("intake getRightArmPosition: " + intake.getRightArmPosition());
    System.out.println("intake getRightArmVelocity: " + intake.getRightArmVelocity());
    System.out.println("intake getRightArmDesiredPosition: " + intake.getRightArmDesiredPosition());
    System.out.println("intake rightArmInPosition: " + intake.rightArmInPosition());
    System.out.println("intake isReady: " + intake.isReady());
    intake.updateDash();

    updateTrajectory();
    System.out.println("calcShooterRPM: " + calcFlywheelRPM());
    System.out.println("calcHoodPosition: " + calcHoodPosition());
    System.out.println("getHubHeading: " + calcShootingHeading());
    updateDash();
  }
}