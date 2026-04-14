package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.TimedRobot;
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
    
    SignalLogger.setPath("/media/sda1/");
    SignalLogger.enableAutoLogging(true);
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    shooter.updateDash();
    indexer.updateDash();
    intake.updateDash();
    updateDash();
  }

  public void autonomousInit() {
    // Runs the init method for each subsystem to reset them to their default states at the start of autonomous. This is important to ensure that there are no unexpected behaviors caused by leftover states.
    indexer.init();
    intake.init();
    shooter.init();

    autoCompleted = true;
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.

      break;

      case auto2:
        // AutoInit 2 code goes here.

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
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    // The following calls are used to update the subsystems and should be called every period.
    indexer.periodic();
    intake.periodic();
    shooter.periodic();

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

  // Publishes information to the dashboard.
  private void updateDash() {
    if (Robot.isSimulation()) SmartDashboard.putNumber("autoStage", autoStage);
  }
}