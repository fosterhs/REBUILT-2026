package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  private double speedScaleFactor = 0.65; // Scales the translational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private double rotationScaleFactor = 0.3; // Scales the rotational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Climber climber = new Climber(); // Initializes the Climber subsystem.
  private final Shooter shooter = new Shooter(); // Initializes the Shooter subsystem.
  private final Indexer indexer = new Indexer(); // Initializes the Indexer subsystem.
  private final Intake intake = new Intake(); // Initializes the Intake subsystem. 

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Auto 1"; 
  private static final String auto2 = "Auto 2"; 
  private String autoSelected;
  private int autoStage = 1;
  private boolean autoCompleted = false;

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Test", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
    SignalLogger.enableAutoLogging(false);
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    climber.updateDash();
    shooter.updateDash();
    indexer.updateDash();
    intake.updateDash();
    updateDash();
  }

  public void autonomousInit() {
    climber.init(); 
    indexer.init();
    intake.init();

    autoCompleted = true;
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.pushCalibration(true, 0.0); // Updates the robot's position on the field.
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.pushCalibration(true, 0.0); // Updates the robot's position on the field.
      break;
    }
  }

  public void autonomousPeriodic() {
    climber.perioidic();
    indexer.periodic();
    intake.periodic();

    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
          break;
        }
      break; 

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
          break;
        }
      break; 
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.
    climber.init(); 
    indexer.init();
    intake.init();
  }

  public void teleopPeriodic() {
    climber.perioidic(); 
    indexer.periodic();
    intake.periodic();

    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    if (driver.getRawButtonPressed(1)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(2)) boostMode = false; // B Button sets default mode (60% of full speed).
    
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*rotationScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) { // X button
      swerveLock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex();
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Left center button
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Updates the position of the robot on the field based on previous calculations.  

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Right center button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.

    if (driver.getPOV() == 0) climber.moveUp(); // D-pad up moves the climber up.
    if (driver.getPOV() == 180) climber.moveDown(); // D-pad down moves the climber down.
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    autoSelected = autoChooser.getSelected();
    if (!autoCompleted) {
      swerve.updateVisionHeading(true, 0.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
    } else {
      swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    }
    swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), true);
  }

  public double getHubHeading() {
    double hubX = 182.11 * 0.0254; // The x-position of the hub on the field in meters.
    double hubY = 158.84 * 0.0254; // The y-position of the hub on the field in meters.
    double robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    double robotY = swerve.getYPos(); // The current y-position of the

    if (robotX > hubX) {
      return Math.toDegrees(Math.atan((hubY - robotY) / (hubX - robotX))) - 90.0; // Returns the heading from the robot to the hub in degrees.
    } else if (robotX < hubX) {
      return Math.toDegrees(Math.atan((hubY - robotY) / (hubX - robotX))) + 90.0; // Returns the heading from the robot to the hub in degrees.
    } else {
      if (robotY > hubY) {
        return 0.0;
      } else {
        return 180.0;
      }
    }
  }

  private double[] distanceArray = { 1.0, 2.0, 5.0, 5.1, 8.5 }; // Distance array (need tested👈)
  private double[] RPMArray = { 2000.0, 3400.0, 3500.0, 4500.0, 5000.0 }; // RPM array(need tested👈)
  public double calculateShooterRPM() {
    double hubX = 182.11 * 0.0254; // The x-position of the hub on the field in meters.
    double hubY = 158.84 * 0.0254; // The y-position of the hub on the field in meters.
    double robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    double robotY = swerve.getYPos(); // The current y-position of the robot
    double distance = Math.sqrt(Math.pow(hubX - robotX, 2) + Math.pow(hubY - robotY, 2)); // distance to hub
    
    if (distance >= distanceArray[distanceArray.length - 1]) {
      return RPMArray[RPMArray.length - 1]; // Return RPM for largest distance
    } 
    else if (distance <= distanceArray[0]) {
      return RPMArray[0]; // Return RPM for smallest distance
    } 
    else {
      int lowerIndex = -1; // Index for distance immediately smaller than current distance
      for (int i = 0; i < distanceArray.length - 1; i++) {
        if (distanceArray[i + 1] > distance && lowerIndex == -1) {
          lowerIndex = i;
        }
      } 
      return RPMArray[lowerIndex] + ((RPMArray[lowerIndex + 1] - RPMArray[lowerIndex]) / (distanceArray[lowerIndex + 1] - distanceArray[lowerIndex])) * (distance - distanceArray[lowerIndex]);
    }
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    SmartDashboard.putBoolean("Boost Mode", boostMode);
    SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() { 
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

    climber.init();
    climber.perioidic();
    climber.moveUp();
    climber.moveDown();
    System.out.println("climber getMode: " + climber.getMode().toString());
    System.out.println("climber atDesiredPosition: " + climber.atDesiredPosition());
    System.out.println("climber getPosition: " + climber.getPosition());
    System.out.println("climber getVelocity: " + climber.getVelocity());
    climber.updateDash();
    
    shooter.spinUp(1000.0);
    shooter.spinDown();
    System.out.println("shooter getRPM: " + shooter.getRPM());
    System.out.println("shooter getRPS: " + shooter.getRPS());
    System.out.println("shooter isAtSpeed(): " + shooter.isAtSpeed());
    System.out.println("shooter isSpunUp: " + shooter.isSpunUp());
    shooter.updateDash();

    indexer.init();
    indexer.periodic();
    indexer.start();
    indexer.jammed();
    indexer.stop();
    System.out.println("indexer getMode: " + indexer.getMode().toString());
    System.out.println("indexer getHopperSensor: " + indexer.getHopperSensor());
    System.out.println("indexer getShooterSensor: " + indexer.getShooterSensor());
    System.out.println("indexer getJamTimer: " + indexer.getJamTimer());
    System.out.println("indexer getShooterTimer: " + indexer.getShooterTimer());
    System.out.println("indexer getHopperTimer: " + indexer.getHopperTimer());
    indexer.updateDash();

    System.out.println("calculateShooterRPM: " + calculateShooterRPM());
    System.out.println("getHubHeading(): " + getHubHeading());
    updateDash();
  }
}