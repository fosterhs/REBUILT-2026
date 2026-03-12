package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class Climber {
  private final CANBus canivore = new CANBus("canivore"); // Creates a new CAN bus called "canivore". This is the name of the CAN bus that the climber motor is connected to. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX climbMotor = new TalonFX(18, canivore); // Creates a new TalonFX motor controller for the climber motor. The motor is assigned an ID of 18 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final StatusSignal<Angle> climberPosition; // Creates a new StatusSignal for the position of the climber motor. This will allow us to read the position of the motor in shaft rotations.
  private final StatusSignal<AngularVelocity> climberVelocity; // Creates a new StatusSignal for the velocity of the climber motor. This will allow us to read the velocity of the motor in shaft rotations per second.
  private final MotionMagicTorqueCurrentFOC climbMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control mode for the climber motor. This will allow us to set the desired position of the motor. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VoltageOut climbMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the climber motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final Timer homingTimer = new Timer(); // Creates a new timer that we will use to time how long the climber has not been moving during the homing process. This will allow us to determine when the climber has stopped moving and can be considered homed.
  private final double upPosition = 90.0; // The position in shaft rotations that corresponds to the climber being fully extended. This will need to be determined experimentally based on your robot's specific climber mechanism.
  private final double downPosition = 25.0; // The position in shaft rotations that corresponds to the climber being fully retracted. This will need to be determined experimentally based on your robot's specific climber mechanism.
  private final double stowPosition = 2.0; // The position in shaft rotations that corresponds to the climber being stowed. This will need to be determined experimentally based on your robot's specific climber mechanism.
  private final double posTol = 1.0; // The tolerance for the position of the climber motor in shaft rotations.
  public enum Mode {HOME, UP, DOWN, STOW} // HOME mode runs the climber motor to find the zero position of the climber. UP mode runs the climber motor to extend the climber to the upPosition. DOWN mode runs the climber motor to retract the climber to the downPosition. STOW mode runs the climber motor to move the climber to the stowPosition.
  public Mode currMode = Mode.HOME; // Initializes the current mode of the climber to HOME. This means that when the robot is first turned on, the climber will be in the process of homing itself to find the zero position.
  private boolean isHomed = false; // A boolean that tracks whether the climber has been homed. This will be set to true once the climber has found the zero position during the HOME mode. The moveUp(), moveDown(), and stow() methods will only work if this is true to prevent the climber from trying to move to a position before it knows where it is.
  private double desiredPosition = 0.0; // A variable to store the desired position of the climber motor in shaft rotations. This will be updated when the moveUp(), moveDown(), and stow() methods are called, and will be used in the atDesiredPosition() method to determine if the climber has reached the desired position.

  // Constructor for the Climber class. This is where we will configure the climber motor with the appropriate settings for our robot. We will set the neutral mode to brake, set the motor direction based on the invert parameter, and configure current limits for the motor. We will also optimize bus utilization for the motor to improve performance. Finally, we will set the update frequency for the climber position and velocity signals to 250 Hz to ensure that we have up-to-date information about the state of the climber motor.
  public Climber() {
    configClimbMotor(climbMotor, false); // Configures the motor with counterclockwise rotation positive.
    climberPosition = climbMotor.getPosition();
    climberVelocity = climbMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, climberPosition, climberVelocity);
    ParentDevice.optimizeBusUtilizationForAll(climbMotor);
  }

  // Resets the climber to the default state: sets the current mode to HOME and restarts the homing timer. Should be called when the robot is enabled to ensure that the climber starts in a known state and begins the homing process.
  public void init() {
    homingTimer.restart();
  }

  // Runs code every 20ms: if the current state is HOME, then run the climber motor at a low voltage to find the zero position of the climber. If the climber has been moving (velocity above a certain threshold), then restart the homing timer. If the climber has not been moving for more than 1 second, then set the position of the motor to 0, set isHomed to true, and switch to STOW mode. If the current state is UP, DOWN, or STOW, then run the climber motor to move to the desired position using MotionMagicTorqueCurrentFOC control mode.
  public void periodic() {
    switch (currMode) {
      case HOME: // Runs the climber motor at a low voltage to find the zero position of the climber. If the climber has been moving (velocity above a certain threshold), then restart the homing timer. If the climber has not been moving for more than 1 second, then set the position of the motor to 0, set isHomed to true, and switch to STOW mode.
        climbMotor.setControl(climbMotorVoltageRequest.withOutput(-2.0).withEnableFOC(true)); // Runs the climber motor at a low voltage to find the zero position of the climber. The negative voltage will make the motor spin in the direction that we want to use for homing, which should be the direction that moves the climber towards its zero position. Adjust this voltage as needed based on your specific robot's climber mechanism and how it responds during testing.
        if (Math.abs(getVelocity()) > 0.5) homingTimer.restart(); // If the climber is moving at a velocity above 0.5 rotations per second, then restart the homing timer. This means that the climber has not yet reached the zero position and is still moving, so we want to keep waiting until it stops.
        if (homingTimer.get() > 1.0) { // If the climber has not been moving for more than 1 second, then we can consider it to be homed at the zero position. We will set the position of the motor to 0, set isHomed to true, and switch to STOW mode.
          climbMotor.setPosition(0.0, 0.03);
          isHomed = true;
          currMode = Mode.STOW;
          desiredPosition = stowPosition;
        }
      break;

      case UP: // Runs the climber motor to move to the desired position using MotionMagicTorqueCurrentFOC control mode.
        climbMotor.setControl(climbMotorPositionRequest.withPosition(desiredPosition)); 
      break;

      case DOWN: // Runs the climber motor to move to the desired position using MotionMagicTorqueCurrentFOC control mode.
        climbMotor.setControl(climbMotorPositionRequest.withPosition(desiredPosition)); 
      break;

      case STOW: // Runs the climber motor to move to the desired position using MotionMagicTorqueCurrentFOC control mode.
        climbMotor.setControl(climbMotorPositionRequest.withPosition(desiredPosition)); 
      break;
    }
  }

  // Sets the current state to UP and updates the desired position to upPosition, which will cause the climber to move to the upPosition in the periodic method. This method will only work if the climber has been homed to prevent trying to move to a position before knowing where it is.
  public void moveUp() {
    if (isHomed) {
      currMode = Mode.UP;
      desiredPosition = upPosition;
    }
  }

  // Sets the current state to DOWN and updates the desired position to downPosition, which will cause the climber to move to the downPosition in the periodic method. This method will only work if the climber has been homed to prevent trying to move to a position before knowing where it is.
  public void moveDown() {
    if (isHomed) {
      currMode = Mode.DOWN;
      desiredPosition = downPosition;
    }
  }

  // Sets the current state to STOW and updates the desired position to stowPosition, which will cause the climber to move to the stowPosition in the periodic method. This method will only work if the climber has been homed to prevent trying to move to a position before knowing where it is.
  public void stow() {
    if (isHomed) {
      currMode = Mode.STOW;
      desiredPosition = stowPosition;
    }
  }

  // Returns the current mode that the climber is in. This can be used for debugging and to determine what the climber is currently trying to do.
  public Mode getMode() {
    return currMode;
  }

  // Returns true if the climber is at the desired position within the specified tolerance. This can be used to determine when the climber has reached its target position and can be useful for sequencing actions that depend on the climber being in a certain position.
  public boolean atDesiredPosition() {
    return Math.abs(desiredPosition - getPosition()) < posTol;
  }

  // Returns the current position of the climber motor in shaft rotations. This is obtained from the climberPosition StatusSignal, which is updated at 250 Hz to ensure that we have up-to-date information about the state of the climber motor.
  public double getPosition() {
    return climberPosition.refresh().getValueAsDouble();
  }

  // Returns the current velocity of the climber motor in shaft rotations per second. This is obtained from the climberVelocity StatusSignal, which is updated at 250 Hz to ensure that we have up-to-date information about the state of the climber motor.
  public double getVelocity() {
    return climberVelocity.refresh().getValueAsDouble();
  }

  // Updates the SmartDashboard with the current mode, position, velocity, and homing status of the climber. Useful for debugging and tuning.
  public void updateDash() {
    //SmartDashboard.putNumber("Climber Timer", homingTimer.get());
    //SmartDashboard.putBoolean("Climber atDesired position", atDesiredPosition());
    //SmartDashboard.putBoolean("Climber isHomed", isHomed);
    //SmartDashboard.putString("Climber Mode", currMode.toString());
    //SmartDashboard.putNumber("Climber Position", getPosition());
    //SmartDashboard.putNumber("Climber Velocity", getVelocity());
  }

  public void simulationPeriodic() {
    
  }

  // A method to configure the climber motor with the appropriate settings for our robot. We will set the neutral mode to brake, set the motor direction based on the invert parameter, and configure current limits for the motor.
  private void configClimbMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise.

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0/18.75; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0/18.75; // Units: amperes per 1 rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/60.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/60.0; // Units: rotations per second.

    // Current limits configuration. These limits will help protect the motor and the mechanism from drawing too much current, which can cause damage. The supply current limit is the maximum current that the motor controller will allow to be drawn from the battery.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 0.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 0.0;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}