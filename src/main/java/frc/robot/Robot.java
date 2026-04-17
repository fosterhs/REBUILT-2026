package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.

// Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.

  // Teleop Driving Variables
  private double xVelTeleop = 0.0; // The x-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double yVelTeleop = 0.0; // The y-velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.
  private double angVelTeleop = 0.0; // The angular velocity of the robot that results from controller inputs after being processed by the slew rate limiters. This is used to control the drivetrain during teleop.

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Auto 1"; 
  private static final String auto2 = "Auto 2"; 
  private String autoSelected;
  private int autoStage = 1;
  private boolean autoCompleted = false;
  
  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.addOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.setDefaultOption(auto1, auto1);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Example", 0.0, 0.0, 0.0, 0.0); // Loads a path generated in PathPlanner into the robot code. 
    
    SignalLogger.setPath("/media/sda1/");
    SignalLogger.enableAutoLogging(true);

    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    updateDash();
  }

  public void autonomousInit() {
    // Runs the init method for each subsystem to reset them to their default states at the start of autonomous. This is important to ensure that there are no unexpected behaviors caused by leftover states.
    swerve.setLimits(1.0, 1.0, 1.0, 1.0);

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
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
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
    swerve.setLimits(1.0, 0.4, 1.0, 1.0); // Sets the speed and acceleration limits for the drivetrain.
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xDemand = MathUtil.applyDeadband(-driver.getLeftY(), 0.05);
    double yDemand = MathUtil.applyDeadband(-driver.getLeftX(), 0.05);
    xVelTeleop = Math.signum(xDemand)*Math.pow(xDemand, 2)*swerve.maxVelSet; // Squaring the total demand allows for finer control at lower speeds while still allowing for full speed at maximum joystick input.
    yVelTeleop = Math.signum(yDemand)*Math.pow(yDemand, 2)*swerve.maxVelSet; // Squaring the total demand allows for finer control at lower speeds while still allowing for full speed at maximum joystick input.
    double totalVel = Math.hypot(xVelTeleop, yVelTeleop);
    if (totalVel > swerve.maxVelSet) { // If the total velocity is greater than the maximum velocity, the velocities are scaled down to maintain the direction of the input while limiting the speed.
      xVelTeleop = xVelTeleop/totalVel*swerve.maxVelSet;
      yVelTeleop = yVelTeleop/totalVel*swerve.maxVelSet;
    }
    double angDemand = Robot.isSimulation() ? MathUtil.applyDeadband(-driver.getRawAxis(2), 0.05) : MathUtil.applyDeadband(-driver.getRightX(), 0.05); 
    angVelTeleop = Math.signum(angDemand)*Math.pow(angDemand, 2)*swerve.maxAngVelSet; // Squaring the total demand allows for finer control at lower speeds while still allowing for full speed at maximum joystick input.

    swerve.drive(xVelTeleop, yVelTeleop, angVelTeleop); // Drive at the velocity demanded by the controller.

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
          swerve.updateVisionHeading(true, 0.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
          if (Robot.isSimulation()) swerve.setSimPose(0.0, 0.0, 0.0); // Sets the simulated position of the robot to match the real position on the field at the start of auto.
        break;

        case auto2:
          swerve.updateVisionHeading(true, 0.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
          if (Robot.isSimulation()) swerve.setSimPose(0.0, 0.0, 0.0); // Sets the simulated position of the robot to match the real position on the field at the start of auto.
        break;
      }
    } else {
      swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    }
    swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), true); 
  }

  public void testInit() {}
  
  public void testPeriodic() {
    swerve.updateOdometry();
    swerve.test(-driver.getLeftY()*swerve.maxVel, Drivetrain.calcHeading(0.0, 0.0 , MathUtil.applyDeadband(-driver.getRightY(), 0.05),  MathUtil.applyDeadband(-driver.getRightX(), 0.05)));
  }

  // Publishes information to the dashboard.
  private void updateDash() {
    if (Robot.isSimulation()) SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  private void runAll() { 
    swerve.resetDriveController(0.0);
    swerve.setLimits(1.0, 1.0, 1.0, 1.0);
    swerve.xLock();
    swerve.test(0.0, 0.0);
    swerve.setSimPose(0.0, 0.0, 0.0);
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
    System.out.println("swerve getAngleDist: " + Drivetrain.getAngleDistance(30.0, -120.0));
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    System.out.println("swerve sterilizeAngle: " + Drivetrain.sterilizeAngle(450.0));
    System.out.println("swerve calcHeading: " + Drivetrain.calcHeading(0.0, 0.0, 1.0, 1.0));
    System.out.println("swerve inerpolate: " + Drivetrain.interpolate(0.0, new double[] {0.0}, new double[] {0.0}));
    swerve.updateDash();
    
    updateDash();
  }
}