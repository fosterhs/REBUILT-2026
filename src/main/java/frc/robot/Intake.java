package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;

public class Intake { 
  // Motors and Sensors
  private final CANBus canivore = new CANBus("canivore"); // Creates a new CAN bus called "canivore". This is the name of the CAN bus that the intake motors are connected to. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX rightArmMotor = new TalonFX(16, canivore); // Creates a new TalonFX motor controller for the right intake arm motor. The motor is assigned an ID of 16 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX rightRollerMotor = new TalonFX(15, canivore); // Creates a new TalonFXS motor controller for the right intake roller motor. The motor is assigned an ID of 15 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX rightCenteringMotor = new TalonFX(20, canivore); // Creates a new TalonFX motor controller for the right intake centering motor. The motor is assigned an ID of 20 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final CANcoder rightArmEncoder = new CANcoder(2, canivore);
  private final TalonFX leftArmMotor = new TalonFX(14, canivore); // Creates a new TalonFX motor controller for the left intake arm motor. The motor is assigned an ID of 14 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX leftRollerMotor = new TalonFX(17, canivore); // Creates a new TalonFXS motor controller for the left intake roller motor. The motor is assigned an ID of 17 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX leftCenteringMotor = new TalonFX(13, canivore); // Creates a new TalonFX motor controller for the left intake centering motor. The motor is assigned an ID of 13 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final CANcoder leftArmEncoder = new CANcoder(1, canivore);

  // Control Requests
  private final MotionMagicTorqueCurrentFOC rightArmPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control request for the right intake arm motor. This will allow us to set a desired position for the motor. 
  private final MotionMagicTorqueCurrentFOC leftArmPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control request for the left intake arm motor. This will allow us to set a desired position for the motor.
  private final MotionMagicVelocityVoltage rightRollerVelocityRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage leftRollerVelocityRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut rightCenteringVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the right intake centering motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut leftCenteringVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the left intake centering motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.

  // Status Signals
  private final StatusSignal<Angle> leftArmPosition; // Creates a new StatusSignal for the position of the left intake arm motor. This will allow us to read the current position of the motor in units of rotations. The TalonFX's integrated encoder provides position feedback that we can use to determine the angle of the intake arm.
  private final StatusSignal<Angle> rightArmPosition; // Creates a new StatusSignal for the position of the right intake arm motor. This will allow us to read the current position of the motor in units of rotations. The TalonFX's integrated encoder provides position feedback that we can use to determine the angle of the intake arm.
  private final double armPosTol = 0.01; // Tolerance for considering the intake arms to be in position at their desired angles. This is in units of motor rotations, so 0.5 means that if the arm is within 0.5 rotations of the desired position, we will consider it to be in position. Adjust this value as needed based on the performance of your specific robot's intake mechanism and how precise you want the positioning to be.
  private final double armStowPosition = 0.0;  // The position that we want to stow the intake arms at when they are not in use. This is in units of motor rotations, so 0.4 means that the arms will be stowed at a position that is 0.4 rotations away from the zero position. Adjust this value as needed based on the physical configuration of your robot's intake mechanism and how you want it to be positioned when stowed.
  private final double armIntakePosition = 0.45; // The position that we want to move the intake arms to when we are intaking fuel. This is in units of motor rotations, so 10.5 means that the arms will move to a position that is 10.5 rotations away from the zero position when intaking. Adjust this value as needed based on the physical configuration of your robot's intake mechanism and how you want it to be positioned when intaking.
  private final double centeringVoltage = 2.0; // Initializes a variable to keep track of the voltage that we want to run the left intake centering motor at when intaking fuel. This can be adjusted based on the performance of your specific robot's intake mechanism and how aggressively you want to run the centering motors to position the intake arm.
  public enum Mode {LEFT, RIGHT, STOW} // HOME mode runs the homing procedure to find the zero position of the intake arms. LEFT mode moves the left arm to the intake position and the right arm to the stow position. RIGHT mode moves the right arm to the intake position and the left arm to the stow position. STOW mode moves both arms to the stow position.
  private Mode currMode = Mode.STOW; // Initializes the current mode of the intake to HOME. This means that when the robot is first turned on, the intake will be in the process of homing to find the zero position of the arms. After homing is complete, it will switch to STOW mode.
  private double desiredLeftArmPosition = armStowPosition; // Initializes a variable to keep track of the desired position for the left intake arm. This will be updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the left arm to move to.
  private double desiredRightArmPosition = armStowPosition; // Initializes a variable to keep track of the desired position for the right intake arm. This will be updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the right arm to move to.
  private boolean leftArmIsStowed = true; // Initializes a boolean variable to keep track of whether the left intake arm is currently in the stowed position. This will be used to determine whether we should run the intake rollers and centering motors, since we only want to run those when the arm is out of the way and ready to intake fuel.
  private boolean rightArmIsStowed = true; // Initializes a boolean variable to keep track of whether the right intake arm is currently in the stowed position. This will be used to determine whether we should run the intake rollers and centering motors, since we only want to run those when the arm is out of the way and ready to intake fuel.
  private double leftIntakeRPM = 5800.0;
  private double rightIntakeRPM = 5800.0;

  // Simulation
  private final TalonFXSimState rightArmMotorSim = rightArmMotor.getSimState();
  private final TalonFXSimState rightRollerMotorSim = rightRollerMotor.getSimState();
  private final TalonFXSimState rightCenteringMotorSim = rightCenteringMotor.getSimState();
  private final TalonFXSimState leftArmMotorSim = leftArmMotor.getSimState();
  private final TalonFXSimState leftRollerMotorSim = leftRollerMotor.getSimState();
  private final TalonFXSimState leftCenteringMotorSim = leftCenteringMotor.getSimState();

  // Constructor for the Intake class. This will be called when we create a new instance of the Intake in our Robot class. In the constructor, we will configure the motors with the appropriate settings for our robot, initialize the status signals for the arm positions and velocities, set the update frequency for those signals, and optimize the CAN bus utilization for all the motors to ensure that we are getting timely updates from all of them without overloading the CAN bus.
  public Intake() {
    configArmMotor(leftArmMotor, leftArmEncoder, true);
    configArmMotor(rightArmMotor, rightArmEncoder, false);
    configRollerMotor(leftRollerMotor, false);
    configRollerMotor(rightRollerMotor, true);
    configCenteringMotor(leftCenteringMotor, true);
    configCenteringMotor(rightCenteringMotor, false);
    configArmEncoder(leftArmEncoder, -0.2893, true);
    configArmEncoder(rightArmEncoder, 0.0573, true);
    leftArmPosition = leftArmEncoder.getAbsolutePosition();
    rightArmPosition = rightArmEncoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, leftArmPosition, rightArmPosition);
    ParentDevice.optimizeBusUtilizationForAll(rightArmMotor, rightRollerMotor, rightCenteringMotor, rightArmEncoder, leftArmMotor, leftRollerMotor, leftCenteringMotor, leftArmEncoder);
  }

  // This method will be called when the robot is first turned on to initialize the intake subsystem. In this method, we will restart both homing timers to ensure that they are starting from zero, and if the arms are already homed, we will call the stow() method to make sure they are in the stow position and ready for operation.
  public void init() {
    currMode = Mode.STOW;
    desiredLeftArmPosition = armStowPosition;
    desiredRightArmPosition = armStowPosition;
    leftArmIsStowed = true;
    rightArmIsStowed = true;
  }

  // This method will be called periodically (about every 20 milliseconds) while the robot is on. In this method, we will implement the logic for controlling the intake arms based on the current mode of the intake system. We will also dynamically adjust the voltage for the rollers and centering motors based on the position of the arms to ensure that we are running them at appropriate speeds for intaking fuel without putting too much strain on the motors or causing excessive wear.
  public void periodic() {
    // Dynamically adjust the roller voltage based on the position of the arms. If the arm is close to the stow position (less than 3 rotations away), we can run the rollers at a lower voltage. If the arm is further out (more than 3 rotations away), we can run the rollers at a higher voltage to more aggressively pull in fuel. Adjust these thresholds and voltage values as needed based on your specific robot's intake mechanism and how it performs during testing.
    leftIntakeRPM = getLeftArmPosition() < 0.1 ? 1800.0 : 5800.0; 
    rightIntakeRPM = getRightArmPosition() < 0.1 ? 1800.0 : 5800.0;

    switch (currMode) {
      case LEFT: // In LEFT mode, we want to move the left arm to the intake position and the right arm to the stow position. We will check if each arm is currently in position at its desired angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = false;
        if (rightArmInPosition()) rightArmIsStowed = true;

        rightArmMotor.setControl(rightArmPositionRequest.withPosition(armStowPosition));
        if (rightArmIsStowed) {
          leftArmMotor.setControl(leftArmPositionRequest.withPosition(armIntakePosition));
        } else {
          leftArmMotor.setControl(leftArmPositionRequest.withPosition(armStowPosition));
        }
      break;

      case RIGHT: // In RIGHT mode, we want to move the right arm to the intake position and the left arm to the stow position. We will check if each arm is currently in position at its desired angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = true;
        if (rightArmInPosition()) rightArmIsStowed = false;

        leftArmMotor.setControl(leftArmPositionRequest.withPosition(armStowPosition));
        if (leftArmIsStowed) {
          rightArmMotor.setControl(rightArmPositionRequest.withPosition(armIntakePosition));
        } else {
          rightArmMotor.setControl(rightArmPositionRequest.withPosition(armStowPosition));
        }
      break;

      case STOW: // In STOW mode, we want to move both arms to the stow position. We will check if each arm is currently in position at the stow angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = true;
        if (rightArmInPosition()) rightArmIsStowed = true;

        leftArmMotor.setControl(leftArmPositionRequest.withPosition(armStowPosition));
        rightArmMotor.setControl(rightArmPositionRequest.withPosition(armStowPosition));
      break;
    }

    // If the arm is stowed, we want to make sure that the rollers and centering motors are not running, since we don't want to run those when the arm is out of the way and not ready to intake fuel. If the arm is not stowed, then we will run the rollers and centering motors at the appropriate voltages that we set earlier in the periodic method. This will ensure that we are only running the rollers and centering motors when the arms are in position and ready to intake fuel, which can help improve the performance of the intake mechanism and reduce wear on the motors.
    if (leftArmIsStowed) {
      leftRollerMotor.setControl(leftRollerVelocityRequest.withVelocity(0.0).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      leftRollerMotor.setControl(leftRollerVelocityRequest.withVelocity(leftIntakeRPM/60.0).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringVoltageRequest.withOutput(centeringVoltage).withEnableFOC(true));
    }
    if (rightArmIsStowed) {
      rightRollerMotor.setControl(rightRollerVelocityRequest.withVelocity(0.0).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      rightRollerMotor.setControl(rightRollerVelocityRequest.withVelocity(rightIntakeRPM/60.0).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringVoltageRequest.withOutput(centeringVoltage).withEnableFOC(true));
    }
  }

  // This method sets the mode of the intake system to LEFT, which will move the left arm to the intake position and the right arm to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to LEFT mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to LEFT, updates the desired positions for both arms to the appropriate intake and stow positions, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at its desired angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void leftIntake() {
    currMode = Mode.LEFT;
    desiredLeftArmPosition = armIntakePosition;
    desiredRightArmPosition = armStowPosition;
    leftArmIsStowed = !leftArmInPosition();
    rightArmIsStowed = rightArmInPosition();
  }

  // This method sets the mode of the intake system to RIGHT, which will move the right arm to the intake position and the left arm to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to RIGHT mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to RIGHT, updates the desired positions for both arms to the appropriate intake and stow positions, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at its desired angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void rightIntake() {
    currMode = Mode.RIGHT;
    desiredLeftArmPosition = armStowPosition;
    desiredRightArmPosition = armIntakePosition;
    leftArmIsStowed = leftArmInPosition();
    rightArmIsStowed = !rightArmInPosition();
  }

  // This method sets the mode of the intake system to STOW, which will move both arms to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to STOW mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to STOW, updates the desired positions for both arms to the armStowPosition, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at the stow angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void stow() {
    currMode = Mode.STOW;
    desiredLeftArmPosition = armStowPosition;
    desiredRightArmPosition = armStowPosition;
    leftArmIsStowed = leftArmInPosition();
    rightArmIsStowed = rightArmInPosition();
  }

  // This method returns the current mode of the intake system. The mode can be one of the following: HOME, LEFT, RIGHT, or STOW. The mode determines how the intake arms are positioned and whether the rollers and centering motors are running. For example, in HOME mode, the system is running the homing procedure to find the zero position of the arms. In LEFT mode, the left arm is moving to the intake position while the right arm is stowed. In RIGHT mode, the right arm is moving to the intake position while the left arm is stowed. In STOW mode, both arms are moving to the stow position. This method allows other parts of the code (such as commands or other subsystems) to check what mode the intake is currently in and make decisions based on that.
  public Mode getMode() {
    return currMode;
  }

  // This method returns the current position of the left intake arm motor by refreshing the leftArmPosition status signal and getting its value as a double. The position is typically measured in units of rotations, and it indicates how far the left intake arm is currently extended or retracted from its zero position. This information can be useful for determining when the arm has reached its desired position during the homing process or when moving to the intake or stow positions.
  public double getLeftArmPosition() {
    return leftArmPosition.refresh().getValueAsDouble();
  }

  // This method returns the desired position for the left intake arm, which is stored in the desiredLeftArmPosition variable. This variable is updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the left arm to move to. For example, in LEFT mode, we want the left arm to be in the intake position, so desiredLeftArmPosition will be set to armIntakePosition. In RIGHT mode, we want the left arm to be in the stow position, so desiredLeftArmPosition will be set to armStowPosition. In STOW mode, we want both arms to be in the stow position, so desiredLeftArmPosition will also be set to armStowPosition.
  public double getLeftArmDesiredPosition() {
    return desiredLeftArmPosition;
  }

  // This method checks if the left intake arm is in position at its desired angle by comparing the current position of the left arm (obtained from the leftArmPosition status signal) to the desired position for the left arm (stored in the desiredLeftArmPosition variable). If the absolute difference between the current position and the desired position is less than the armPosTol tolerance value, then we consider the left arm to be in position and the method returns true. Otherwise, it returns false, indicating that the left arm is not yet in position at its desired angle.
  public boolean leftArmInPosition() {
    return Math.abs(getLeftArmPosition() - getLeftArmDesiredPosition()) < armPosTol;
  }

  // This method returns the current position of the right intake arm motor by refreshing the rightArmPosition status signal and getting its value as a double. The position is typically measured in units of rotations, and it indicates how far the right intake arm is currently extended or retracted from its zero position. This information can be useful for determining when the arm has reached its desired position during the homing process or when moving to the intake or stow positions.
  public double getRightArmPosition() {
    return rightArmPosition.refresh().getValueAsDouble();
  }

  // This method returns the desired position for the right intake arm, which is stored in the desiredRightArmPosition variable. This variable is updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the right arm to move to. For example, in LEFT mode, we want the right arm to be in the stow position, so desiredRightArmPosition will be set to armStowPosition. In RIGHT mode, we want the right arm to be in the intake position, so desiredRightArmPosition will be set to armIntakePosition. In STOW mode, we want both arms to be in the stow position, so desiredRightArmPosition will also be set to armStowPosition.
  public double getRightArmDesiredPosition() {
    return desiredRightArmPosition;
  }
  
  // This method checks if the right intake arm is in position at its desired angle by comparing the current position of the right arm (obtained from the rightArmPosition status signal) to the desired position for the right arm (stored in the desiredRightArmPosition variable). If the absolute difference between the current position and the desired position is less than the armPosTol tolerance value, then we consider the right arm to be in position and the method returns true. Otherwise, it returns false, indicating that the right arm is not yet in position at its desired angle.
  public boolean rightArmInPosition() {
    return Math.abs(getRightArmPosition() - getRightArmDesiredPosition()) < armPosTol;
  }

  // This method checks if the intake is ready to intake fuel by verifying that both arms are in position at their desired angles. It returns true if both arms are in position and the current mode is either LEFT or RIGHT (indicating that we are trying to intake fuel), and false otherwise.
  public boolean isReady() {
    if (getMode() == Mode.LEFT || getMode() == Mode.RIGHT) {
      return leftArmInPosition() && rightArmInPosition();
    } else {
      return false;
    }
  }

  // This method updates the SmartDashboard with various status information about the intake subsystem. This can be useful for debugging and monitoring the performance of the intake during operation. The information being displayed includes the current mode of the intake, the positions and velocities of the left and right intake arms, whether each arm is in position at its desired angle, and whether the intake is ready to intake fuel (i.e., both arms are in position). You can uncomment these lines to see this information on the SmartDashboard while running your robot.
  public void updateDash() {
    //SmartDashboard.putString("Intake getMode", currMode.toString());
    //SmartDashboard.putNumber("Intake getLeftArmPosition", getLeftArmPosition());
    //SmartDashboard.putNumber("Intake getLeftArmDesiredPosition", getLeftArmDesiredPosition());
    //SmartDashboard.putBoolean("Intake leftArmInPosition", leftArmInPosition());
    //SmartDashboard.putNumber("Intake getRightArmPosition", getRightArmPosition());
    //SmartDashboard.putNumber("Intake getRightArmDesiredPosition", getRightArmDesiredPosition());
    //SmartDashboard.putBoolean("Intake rightArmInPosition", rightArmInPosition());
    //SmartDashboard.putBoolean("Intake isReady", isReady());
  }

  public void simulationPeriodic() {
    // TODO: update this code
    // TalonFX Motor Sims
    // rightArmMotorSim
    // rightRollerMotorSim
    // rightCenteringMotorSim
    // leftArmMotorSim
    // leftRollerMotorSim
    // leftCenteringMotorSim

    // Simulate homing, simulate fully extended
  }

  // This method configures the settings for the roller motors. The configuration includes setting the commutation settings specific to the TalonFXS, setting the neutral mode to brake, configuring the motor direction based on the invert parameter, and configuring current limits to protect the motors and mechanical components of the intake mechanism. Finally, it applies the configuration to the motor controller with a specified timeout.
  private void configRollerMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Start with a new configuration object for the roller motors. The TalonFXS has some different configuration options compared to the TalonFX, so we use a different configuration class.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set the neutral mode to brake so that the motors will resist movement when no power is applied. This can help hold the rollers in position when we want them to stay still.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Set the motor direction based on the invert parameter. This allows us to easily configure one roller to be inverted and the other to be non-inverted, which can help ensure that they move in the correct directions when we apply positive or negative voltages to them.

    // MotionMagicVelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.20; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 6.0; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.012; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/60.0; // Units: roations per second.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 8.0*5800.0/60.0; // Units: rotations per second per second. 

    // Current limits configuration. These limits can help protect the motors and the mechanical components of the intake from drawing too much current and potentially causing damage. Adjust these values as needed based on the performance of your specific robot's intake mechanism and the capabilities of your motors.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 200.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Apply the configuration to the motor with a timeout of 0.03 seconds (30 milliseconds). This will send the configuration settings to the motor controller so that it can use them for controlling the motor.
  }

  // This method configures the settings for the centering motors. The configuration includes setting the neutral mode to brake, configuring the motor direction based on the invert parameter, and configuring current limits to protect the motors and mechanical components of the intake mechanism. Finally, it applies the configuration to the motor controller with a specified timeout.
  private void configCenteringMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Start with a new configuration object for the centering motors.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set the neutral mode to brake so that the motors will resist movement when no power is applied. This can help hold the centering mechanism in position when we want it to stay still.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Set the motor direction based on the invert parameter. This allows us to easily configure one centering motor to be inverted and the other to be non-inverted, which can help ensure that they move in the correct directions when we apply positive or negative voltages to them.

    // Current limits configuration. These limits can help protect the motors and the mechanical components of the intake from drawing too much current and potentially causing damage. Adjust these values as needed based on the performance of your specific robot's intake mechanism and the capabilities of your motors.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 10.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 60.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Apply the configuration to the motor with a timeout of 0.03 seconds (30 milliseconds). This will send the configuration settings to the motor controller so that it can use them for controlling the motor.
  }

  // This method configures the settings for the arm motors. The configuration includes setting the neutral mode to brake, configuring the motor direction based on the invert parameter, setting up the PIDF constants for MotionMagicTorqueFOC closed-loop control, and configuring current limits to protect the motors and mechanical components of the intake mechanism. Finally, it applies the configuration to the motor controller with a specified timeout.
  private void configArmMotor(TalonFX motor, CANcoder encoder, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Start with a new configuration object for the arm motors.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set the neutral mode to brake so that the motors will resist movement when no power is applied. This can help hold the arms in position when we want them to stay still.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Set the motor direction based on the invert parameter. This allows us to easily configure one arm to be inverted and the other to be non-inverted, which can help ensure that they move in the correct directions when we apply positive or negative voltages to them.

    motorConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID(); // Sets the ID of the remote sensor that will be used for feedback in closed-loop control. In this case, we are using the hood encoder as the feedback device for the hood motor, so we set this to the device ID of the hood encoder.
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // Sets the source of the feedback sensor for closed-loop control. Since we are using the hood encoder as the feedback device, we set this to FusedCANcoder, which means that the motor will use both the integrated sensor and the remote CANcoder sensor for feedback.
    motorConfigs.Feedback.RotorToSensorRatio = 25.0; // The ratio of motor rotations to sensor rotations. This should be set based on the gearing between the motor and the hood mechanism.

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0*25.0/18.75; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 80.0*25.0/18.75; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 5.0*5800.0/(60.0*25.0); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*25.0); // Units: roations per second.

    // Current limits configuration. These limits can help protect the motors and the mechanical components of the intake from drawing too much current and potentially causing damage. Adjust these values as needed based on the performance of your specific robot's intake mechanism and the capabilities of your motors.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 10.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Apply the configuration to the motor with a timeout of 0.03 seconds (30 milliseconds). This will send the configuration settings to the motor controller so that it can use them for controlling the motor.
  }

  // Configures the hood encoder with the appropriate settings for our robot. Sets the absolute sensor discontinuity point, magnet offset, and sensor direction based on the physical configuration of the encoder on our robot. These settings are important to ensure that the encoder readings are accurate and consistent with the actual position of the hood.
  private void configArmEncoder(CANcoder encoder, double offset, boolean invert) {
    CANcoderConfiguration sensorConfigs = new CANcoderConfiguration(); // Creates a new configuration object for the CANcoder. This object will hold all the settings that we want to apply to the encoder.

    sensorConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // The point at which the sensor's absolute position reading discontinuously jumps from 1.0 back to 0.0. This should be set based on the physical configuration of the encoder on our robot. 
    sensorConfigs.MagnetSensor.MagnetOffset = offset; // The offset to apply to the sensor's absolute position reading to align it with the actual position of the hood. This should be set based on the physical configuration of the encoder on our robot.
    sensorConfigs.MagnetSensor.SensorDirection = invert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive; // The direction that the sensor counts as positive. This should be set based on the physical configuration of the encoder on our robot.

    encoder.getConfigurator().apply(sensorConfigs, 0.03); // Applies the configuration to the encoder with a timeout of 0.03 seconds. This will set all the settings that we specified in the sensorConfigs object to the encoder.
  }
}