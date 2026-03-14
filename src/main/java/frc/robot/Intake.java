package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class Intake { 
  // Motors
  private final CANBus canivore = new CANBus("canivore"); // Creates a new CAN bus called "canivore". This is the name of the CAN bus that the intake motors are connected to. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX rightArmMotor = new TalonFX(16, canivore); // Creates a new TalonFX motor controller for the right intake arm motor. The motor is assigned an ID of 16 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFXS rightRollerMotor = new TalonFXS(15, canivore); // Creates a new TalonFXS motor controller for the right intake roller motor. The motor is assigned an ID of 15 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX rightCenteringMotor = new TalonFX(20, canivore); // Creates a new TalonFX motor controller for the right intake centering motor. The motor is assigned an ID of 20 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX leftArmMotor = new TalonFX(14, canivore); // Creates a new TalonFX motor controller for the left intake arm motor. The motor is assigned an ID of 14 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFXS leftRollerMotor = new TalonFXS(17, canivore); // Creates a new TalonFXS motor controller for the left intake roller motor. The motor is assigned an ID of 17 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX leftCenteringMotor = new TalonFX(13, canivore); // Creates a new TalonFX motor controller for the left intake centering motor. The motor is assigned an ID of 13 on the CAN bus. Make sure to set this correctly based on your robot's wiring.

  // Control Requests
  private final MotionMagicTorqueCurrentFOC rightArmMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control request for the right intake arm motor. This will allow us to set a desired position for the motor. 
  private final MotionMagicTorqueCurrentFOC leftArmMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control request for the left intake arm motor. This will allow us to set a desired position for the motor.
  private final VoltageOut rightArmMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the right intake arm motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut leftArmMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the left intake arm motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut rightRollerMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the right intake roller motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut leftRollerMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the left intake roller motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut rightCenteringMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the right intake centering motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut leftCenteringMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the left intake centering motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.

  // Status Signals
  private final StatusSignal<Angle> leftArmPosition; // Creates a new StatusSignal for the position of the left intake arm motor. This will allow us to read the current position of the motor in units of rotations. The TalonFX's integrated encoder provides position feedback that we can use to determine the angle of the intake arm.
	private final StatusSignal<AngularVelocity> leftArmVelocity; // Creates a new StatusSignal for the velocity of the left intake arm motor. This will allow us to read the current velocity of the motor in units of rotations per second. The TalonFX's integrated encoder provides velocity feedback that we can use to determine how fast the intake arm is moving.
  private final StatusSignal<Angle> rightArmPosition; // Creates a new StatusSignal for the position of the right intake arm motor. This will allow us to read the current position of the motor in units of rotations. The TalonFX's integrated encoder provides position feedback that we can use to determine the angle of the intake arm.
	private final StatusSignal<AngularVelocity> rightArmVelocity; // Creates a new StatusSignal for the velocity of the right intake arm motor. This will allow us to read the current velocity of the motor in units of rotations per second. The TalonFX's integrated encoder provides velocity feedback that we can use to determine how fast the intake arm is moving.

  private final Timer leftHomingTimer = new Timer(); // Creates a new timer that we will use to time how long the left intake arm has been stationary during the homing process. This will allow us to determine when the arm has stopped moving and can be considered homed at the zero position.
  private final Timer rightHomingTimer = new Timer(); // Creates a new timer that we will use to time how long the right intake arm has been stationary during the homing process. This will allow us to determine when the arm has stopped moving and can be considered homed at the zero position.
  private final double armPosTol = 0.5; // Tolerance for considering the intake arms to be in position at their desired angles. This is in units of motor rotations, so 0.5 means that if the arm is within 0.5 rotations of the desired position, we will consider it to be in position. Adjust this value as needed based on the performance of your specific robot's intake mechanism and how precise you want the positioning to be.
  private final double armStowPosition = 0.4;  // The position that we want to stow the intake arms at when they are not in use. This is in units of motor rotations, so 0.4 means that the arms will be stowed at a position that is 0.4 rotations away from the zero position. Adjust this value as needed based on the physical configuration of your robot's intake mechanism and how you want it to be positioned when stowed.
  private final double armIntakePosition = 10.5; // The position that we want to move the intake arms to when we are intaking fuel. This is in units of motor rotations, so 10.5 means that the arms will move to a position that is 10.5 rotations away from the zero position when intaking. Adjust this value as needed based on the physical configuration of your robot's intake mechanism and how you want it to be positioned when intaking.
  public enum Mode {HOME, LEFT, RIGHT, STOW} // HOME mode runs the homing procedure to find the zero position of the intake arms. LEFT mode moves the left arm to the intake position and the right arm to the stow position. RIGHT mode moves the right arm to the intake position and the left arm to the stow position. STOW mode moves both arms to the stow position.
  private Mode currMode = Mode.HOME; // Initializes the current mode of the intake to HOME. This means that when the robot is first turned on, the intake will be in the process of homing to find the zero position of the arms. After homing is complete, it will switch to STOW mode.
  private boolean leftArmIsHomed = false; // Initializes a boolean variable to keep track of whether the left intake arm has been homed. This will be set to true once the homing procedure for the left arm is complete and we have established the zero position for that arm.
  private boolean rightArmIsHomed = false; // Initializes a boolean variable to keep track of whether the right intake arm has been homed. This will be set to true once the homing procedure for the right arm is complete and we have established the zero position for that arm.
  private boolean isHomed = false; // Initializes a boolean variable to keep track of whether both intake arms have been homed.
  private double desiredLeftArmPosition = 0.0; // Initializes a variable to keep track of the desired position for the left intake arm. This will be updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the left arm to move to.
  private double desiredRightArmPosition = 0.0; // Initializes a variable to keep track of the desired position for the right intake arm. This will be updated based on the current mode of the intake (LEFT, RIGHT, or STOW) to determine where we want the right arm to move to.
  private boolean leftArmIsStowed = true; // Initializes a boolean variable to keep track of whether the left intake arm is currently in the stowed position. This will be used to determine whether we should run the intake rollers and centering motors, since we only want to run those when the arm is out of the way and ready to intake fuel.
  private boolean rightArmIsStowed = true; // Initializes a boolean variable to keep track of whether the right intake arm is currently in the stowed position. This will be used to determine whether we should run the intake rollers and centering motors, since we only want to run those when the arm is out of the way and ready to intake fuel.
  private double leftRollerVoltage = 6.0; // Initializes a variable to keep track of the voltage that we want to run the left intake roller motor at when intaking fuel. This can be adjusted based on the performance of your specific robot's intake mechanism and how aggressively you want to run the rollers to pull in fuel. We will also adjust this voltage dynamically in the periodic method based on the position of the arms.
  private double rightRollerVoltage = 6.0; // Initializes a variable to keep track of the voltage that we want to run the right intake roller motor at when intaking fuel. This can be adjusted based on the performance of your specific robot's intake mechanism and how aggressively you want to run the rollers to pull in fuel. We will also adjust this voltage dynamically in the periodic method based on the position of the arms.
  private double leftCenteringVoltage = 2.0; // Initializes a variable to keep track of the voltage that we want to run the left intake centering motor at when intaking fuel. This can be adjusted based on the performance of your specific robot's intake mechanism and how aggressively you want to run the centering motors to position the intake arm.
  private double rightCenteringVoltage = 2.0; // Initializes a variable to keep track of the voltage that we want to run the right intake centering motor at when intaking fuel. This can be adjusted based on the performance of your specific robot's intake mechanism and how aggressively you want to run the centering motors to position the intake arm.

  // Simulation
  private final TalonFXSimState rightArmMotorSim = rightArmMotor.getSimState();
  private final TalonFXSSimState rightRollerMotorSim = rightRollerMotor.getSimState();
  private final TalonFXSimState rightCenteringMotorSim = rightCenteringMotor.getSimState();
  private final TalonFXSimState leftArmMotorSim = leftArmMotor.getSimState();
  private final TalonFXSSimState leftRollerMotorSim = leftRollerMotor.getSimState();
  private final TalonFXSimState leftCenteringMotorSim = leftCenteringMotor.getSimState();

  // Constructor for the Intake class. This will be called when we create a new instance of the Intake in our Robot class. In the constructor, we will configure the motors with the appropriate settings for our robot, initialize the status signals for the arm positions and velocities, set the update frequency for those signals, and optimize the CAN bus utilization for all the motors to ensure that we are getting timely updates from all of them without overloading the CAN bus.
  public Intake() {
    configArmMotor(leftArmMotor, true);
    configArmMotor(rightArmMotor, false);
    configRollerMotor(leftRollerMotor, false);
    configRollerMotor(rightRollerMotor, true);
    configCenteringMotor(leftCenteringMotor, true);
    configCenteringMotor(rightCenteringMotor, false);
    leftArmPosition = leftArmMotor.getPosition();
    leftArmVelocity = leftArmMotor.getVelocity();
    rightArmPosition = rightArmMotor.getPosition();
    rightArmVelocity = rightArmMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, leftArmPosition, leftArmVelocity, rightArmPosition, rightArmVelocity);
    ParentDevice.optimizeBusUtilizationForAll(rightArmMotor, rightRollerMotor, rightCenteringMotor, leftArmMotor, leftRollerMotor, leftCenteringMotor);
  }

  // This method will be called when the robot is first turned on to initialize the intake subsystem. In this method, we will restart both homing timers to ensure that they are starting from zero, and if the arms are already homed, we will call the stow() method to make sure they are in the stow position and ready for operation.
  public void init() {
    leftHomingTimer.restart();
    rightHomingTimer.restart();
    if (isHomed) stow();
  }

  // This method will be called periodically (about every 20 milliseconds) while the robot is on. In this method, we will implement the logic for controlling the intake arms based on the current mode of the intake system. We will also dynamically adjust the voltage for the rollers and centering motors based on the position of the arms to ensure that we are running them at appropriate speeds for intaking fuel without putting too much strain on the motors or causing excessive wear.
  public void periodic() {
    // Dynamically adjust the roller voltage based on the position of the arms. If the arm is close to the stow position (less than 3 rotations away), we can run the rollers at a lower voltage. If the arm is further out (more than 3 rotations away), we can run the rollers at a higher voltage to more aggressively pull in fuel. Adjust these thresholds and voltage values as needed based on your specific robot's intake mechanism and how it performs during testing.
    leftRollerVoltage = getLeftArmPosition() < 3.0 ? 4.0 : 6.0; 
    rightRollerVoltage = getRightArmPosition() < 3.0 ? 4.0 : 6.0;

    switch (currMode) {
      case HOME: // In HOME mode, we want to run the homing procedure to find the zero position of the intake arms. We will run the arm motors at a low voltage to move them towards the zero position, and if they are moving, we will restart the homing timers. If they have been stationary for more than 1 second, we will set their positions to 0 and consider them homed. Once both arms are homed, we will switch to STOW mode and set the desired positions for both arms to the stow position.
        if (Robot.isSimulation()) {
          currMode = Mode.STOW;
        }
        
        // Runs the arm motors at a low voltage to find the zero position. The negative voltage will make the motor spin in the direction that we want to use for homing, which should be the direction that moves the arm towards its zero position. Adjust this voltage as needed based on your specific robot's intake mechanism and how it responds during testing.
        leftArmMotor.setControl(leftArmMotorVoltageRequest.withOutput(-1.0).withEnableFOC(true));
        rightArmMotor.setControl(rightArmMotorVoltageRequest.withOutput(-1.0).withEnableFOC(true));

        // If the arms are moving at a velocity above 0.5 rotations per second, then restart the homing timers. This means that the arms have not yet reached the zero position and are still moving, so we want to keep waiting until they stop.
        if (Math.abs(getLeftArmVelocity()) > 2.0) leftHomingTimer.restart();
        if (Math.abs(getRightArmVelocity()) > 2.0) rightHomingTimer.restart();

        // If the arms have not been homed yet and have been stationary for more than 1 second, then we will set the position of the arm motor to 0. This will establish the zero position for the arm. 
        if (leftHomingTimer.get() > 1.0 && !leftArmIsHomed) { 
          leftArmMotor.setPosition(0.0, 0.03);
          leftArmIsHomed = true;
          leftArmIsStowed = true;
        }
        if (rightHomingTimer.get() > 1.0 && !rightArmIsHomed) {
          rightArmMotor.setPosition(0.0, 0.03);
          rightArmIsHomed = true;
          rightArmIsStowed = true;
        }

        // If both arms are homed, then we can switch to STOW mode and set the desired positions for both arms to the stow position. This will ensure that once homing is complete, the arms will move to the stow position and be ready for operation.
        if (leftArmIsHomed && rightArmIsHomed) {
          isHomed = true;
          currMode = Mode.STOW;
          desiredLeftArmPosition = armStowPosition;
          desiredRightArmPosition = armStowPosition;
        }
      break;

      case LEFT: // In LEFT mode, we want to move the left arm to the intake position and the right arm to the stow position. We will check if each arm is currently in position at its desired angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = false;
        if (rightArmInPosition()) rightArmIsStowed = true;

        rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
        if (rightArmIsStowed) {
          leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armIntakePosition));
        } else {
          leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        }
      break;

      case RIGHT: // In RIGHT mode, we want to move the right arm to the intake position and the left arm to the stow position. We will check if each arm is currently in position at its desired angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = true;
        if (rightArmInPosition()) rightArmIsStowed = false;

        leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        if (leftArmIsStowed) {
          rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armIntakePosition));
        } else {
          rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
        }
      break;

      case STOW: // In STOW mode, we want to move both arms to the stow position. We will check if each arm is currently in position at the stow angle, and update the leftArmIsStowed and rightArmIsStowed variables accordingly. This will allow us to know whether we should run the rollers and centering motors for each arm based on whether they are stowed or not.
        if (leftArmInPosition()) leftArmIsStowed = true;
        if (rightArmInPosition()) rightArmIsStowed = true;

        leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
      break;
    }

    // If the arm is stowed, we want to make sure that the rollers and centering motors are not running, since we don't want to run those when the arm is out of the way and not ready to intake fuel. If the arm is not stowed, then we will run the rollers and centering motors at the appropriate voltages that we set earlier in the periodic method. This will ensure that we are only running the rollers and centering motors when the arms are in position and ready to intake fuel, which can help improve the performance of the intake mechanism and reduce wear on the motors.
    if (leftArmIsStowed) {
      leftRollerMotor.setControl(leftRollerMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      leftRollerMotor.setControl(leftRollerMotorVoltageRequest.withOutput(leftRollerVoltage).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringMotorVoltageRequest.withOutput(leftCenteringVoltage).withEnableFOC(true));
    }
    if (rightArmIsStowed) {
      rightRollerMotor.setControl(rightRollerMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      rightRollerMotor.setControl(rightRollerMotorVoltageRequest.withOutput(rightRollerVoltage).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringMotorVoltageRequest.withOutput(rightCenteringVoltage).withEnableFOC(true));
    }
  }

  // This method sets the mode of the intake system to HOME, which will start the homing procedure to find the zero position of the intake arms. It resets the desired positions for both arms to 0.0, sets the leftArmIsStowed and rightArmIsStowed variables to false since we are in the process of homing and want to run the rollers and centering motors, sets the leftArmIsHomed and rightArmIsHomed variables to false since we haven't completed homing yet, and restarts both homing timers to start timing how long each arm has been stationary during the homing process.
  public void home() {
    currMode = Mode.HOME;
    isHomed = false;
    desiredLeftArmPosition = 0.0;
    desiredRightArmPosition = 0.0;
    leftArmIsStowed = false;
    rightArmIsStowed = false;
    leftArmIsHomed = false;
    rightArmIsHomed = false;
    leftHomingTimer.restart();
    rightHomingTimer.restart();
  }

  // This method sets the mode of the intake system to LEFT, which will move the left arm to the intake position and the right arm to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to LEFT mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to LEFT, updates the desired positions for both arms to the appropriate intake and stow positions, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at its desired angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void leftIntake() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.LEFT;
      desiredLeftArmPosition = armIntakePosition;
      desiredRightArmPosition = armStowPosition;
      leftArmIsStowed = !leftArmInPosition();
      rightArmIsStowed = rightArmInPosition();
    }
  }

  // This method sets the mode of the intake system to RIGHT, which will move the right arm to the intake position and the left arm to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to RIGHT mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to RIGHT, updates the desired positions for both arms to the appropriate intake and stow positions, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at its desired angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void rightIntake() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.RIGHT;
      desiredLeftArmPosition = armStowPosition;
      desiredRightArmPosition = armIntakePosition;
      leftArmIsStowed = leftArmInPosition();
      rightArmIsStowed = !rightArmInPosition();
    }
  }

  // This method sets the mode of the intake system to STOW, which will move both arms to the stow position. It first checks if the current mode is not HOME, since we don't want to allow switching to STOW mode while we are still in the process of homing the arms. If we are not in HOME mode, it sets the current mode to STOW, updates the desired positions for both arms to the armStowPosition, and updates the leftArmIsStowed and rightArmIsStowed variables based on whether each arm is currently in position at the stow angle. This will ensure that the periodic method knows whether to run the rollers and centering motors based on whether the arms are stowed or not.
  public void stow() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.STOW;
      desiredLeftArmPosition = armStowPosition;
      desiredRightArmPosition = armStowPosition;
      leftArmIsStowed = leftArmInPosition();
      rightArmIsStowed = rightArmInPosition();
    }
  }

  // This method returns the current mode of the intake system. The mode can be one of the following: HOME, LEFT, RIGHT, or STOW. The mode determines how the intake arms are positioned and whether the rollers and centering motors are running. For example, in HOME mode, the system is running the homing procedure to find the zero position of the arms. In LEFT mode, the left arm is moving to the intake position while the right arm is stowed. In RIGHT mode, the right arm is moving to the intake position while the left arm is stowed. In STOW mode, both arms are moving to the stow position. This method allows other parts of the code (such as commands or other subsystems) to check what mode the intake is currently in and make decisions based on that.
  public Mode getMode() {
    return currMode;
  }

  // This method returns the current position of the left intake arm motor by refreshing the leftArmPosition status signal and getting its value as a double. The position is typically measured in units of rotations, and it indicates how far the left intake arm is currently extended or retracted from its zero position. This information can be useful for determining when the arm has reached its desired position during the homing process or when moving to the intake or stow positions.
  public double getLeftArmPosition() {
    return leftArmPosition.refresh().getValueAsDouble();
  }

  // This method returns the current velocity of the left intake arm motor by refreshing the leftArmVelocity status signal and getting its value as a double. The velocity is typically measured in units of rotations per second, and it indicates how fast the left intake arm is currently moving. This information can be useful for determining when the arm has stopped moving during the homing process, or for monitoring the performance of the arm during operation.
  public double getLeftArmVelocity() {
    return leftArmVelocity.refresh().getValueAsDouble();
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

  // This method returns the current velocity of the right intake arm motor by refreshing the rightArmVelocity status signal and getting its value as a double. The velocity is typically measured in units of rotations per second, and it indicates how fast the right intake arm is currently moving. This information can be useful for determining when the arm has stopped moving during the homing process, or for monitoring the performance of the arm during operation.
  public double getRightArmVelocity() {
    return rightArmVelocity.refresh().getValueAsDouble();
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
    //SmartDashboard.putNumber("Intake getLeftArmVelocity", getLeftArmVelocity());
    //SmartDashboard.putNumber("Intake getLeftArmDesiredPosition", getLeftArmDesiredPosition());
    //SmartDashboard.putBoolean("Intake leftArmInPosition", leftArmInPosition());
    //SmartDashboard.putNumber("Intake getRightArmPosition", getRightArmPosition());
    //SmartDashboard.putNumber("Intake getRightArmVelocity", getRightArmVelocity());
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
  private void configRollerMotor(TalonFXS motor, boolean invert) {
    TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration(); // Start with a new configuration object for the roller motors. The TalonFXS has some different configuration options compared to the TalonFX, so we use a different configuration class.

    motorConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST; // Set the motor arrangement to Minion_JST, which is a specific configuration for the internal wiring of the motor. This is important to set correctly based on the type of motor you are using and how it is wired, as it affects how the motor controller will drive the motor and read feedback from it.
    motorConfigs.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled; // Enable advanced hall support, which can help improve the performance of the motor by providing better feedback.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set the neutral mode to brake so that the motors will resist movement when no power is applied. This can help hold the rollers in position when we want them to stay still.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Set the motor direction based on the invert parameter. This allows us to easily configure one roller to be inverted and the other to be non-inverted, which can help ensure that they move in the correct directions when we apply positive or negative voltages to them.

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
  private void configArmMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Start with a new configuration object for the arm motors.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set the neutral mode to brake so that the motors will resist movement when no power is applied. This can help hold the arms in position when we want them to stay still.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Set the motor direction based on the invert parameter. This allows us to easily configure one arm to be inverted and the other to be non-inverted, which can help ensure that they move in the correct directions when we apply positive or negative voltages to them.

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0/18.75; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0/18.75; // Units: amperes per 1 rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 5800.0/60.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/60.0; // Units: rotations per second.

    // Current limits configuration. These limits can help protect the motors and the mechanical components of the intake from drawing too much current and potentially causing damage. Adjust these values as needed based on the performance of your specific robot's intake mechanism and the capabilities of your motors.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 80.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Apply the configuration to the motor with a timeout of 0.03 seconds (30 milliseconds). This will send the configuration settings to the motor controller so that it can use them for controlling the motor.
  }
}