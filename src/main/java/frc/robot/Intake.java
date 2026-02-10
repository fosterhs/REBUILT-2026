
package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
  public enum Mode {HOME, LEFT, RIGHT, STOW}
  public Mode currMode = Mode.HOME;
  public double desiredLeftArmPosition = 0.0;
  public double desiredLeftRollerVelocity = 0.0;
  public double desiredRightArmPosition = 0.0;
  public double desiredRightRollerVelocity = 0.0;
  
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX rightIntakeDeploy = new TalonFX(14, canivore); // Right arm motor
  private final TalonFX rightIntake = new TalonFX(15, canivore); // Right roller motor
  private final TalonFX leftIntakeDeploy = new TalonFX(16, canivore); // Left arm motor
  private final TalonFX leftIntake = new TalonFX(17, canivore); // Left roller motor
  private final Timer intakeTimer = new Timer();
  
  // Control requests
  private final PositionVoltage armPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  public Intake() {
    configMotor(rightIntakeDeploy, true, 40.0, true);  // Arm motor
    configMotor(rightIntake, false, 60.0, false);      // Roller motor
    configMotor(leftIntakeDeploy, false, 40.0, true);  // Arm motor
    configMotor(leftIntake, true, 60.0, false);        // Roller motor
    intakeTimer.start();
  }

  public void init() {
    currMode = Mode.HOME;
    desiredLeftArmPosition = 0.0;
    desiredLeftRollerVelocity = 0.0;
    desiredRightArmPosition = 0.0;
    desiredRightRollerVelocity = 0.0;
    intakeTimer.reset();
  }

  public void periodic() {
    switch (currMode) {
      case HOME:
        desiredLeftArmPosition = 0.0;
        desiredLeftRollerVelocity = 0.0;
        desiredRightArmPosition = 0.0;
        desiredRightRollerVelocity = 0.0;
        break;

      case LEFT:
        desiredLeftArmPosition = 2.0;
        desiredLeftRollerVelocity = 10.0;
        desiredRightArmPosition = 0.0;
        desiredRightRollerVelocity = 0.0;
        break;

      case RIGHT:
        desiredLeftArmPosition = 0.0;
        desiredLeftRollerVelocity = 0.0;
        desiredRightArmPosition = 2.0;
        desiredRightRollerVelocity = 10.0;
        break;

      case STOW:
        desiredLeftArmPosition = 0.5;
        desiredLeftRollerVelocity = 0.0;
        desiredRightArmPosition = 0.5; 
        desiredRightRollerVelocity = 0.0;
        break;
    }
    
    // Apply desired positions/velocities
    leftIntakeDeploy.setControl(armPositionRequest.withPosition(desiredLeftArmPosition));
    leftIntake.setControl(rollerVelocityRequest.withVelocity(desiredLeftRollerVelocity));
    rightIntakeDeploy.setControl(armPositionRequest.withPosition(desiredRightArmPosition));
    rightIntake.setControl(rollerVelocityRequest.withVelocity(desiredRightRollerVelocity));
  }

  // Activates the left intake: deploys the left arm and turns on the left roller. Resets the intake timer to prevent automatic transitions to other modes.
  public void leftIntake() {
    currMode = Mode.LEFT;
    intakeTimer.reset();
  }

  // Activates the right intake: deploys the right arm and turns on the right roller. Resets the intake timer to prevent automatic transitions to other modes.
  public void rightIntake() {
    currMode = Mode.RIGHT;
    intakeTimer.reset();
  }

  // Stows the intake arms and stops the rollers. Resets the intake timer to prevent automatic transitions to other modes.
  public void stowIntake() {
    currMode = Mode.STOW;
    intakeTimer.reset();
  }

  // Returns the current mode of the intake.
  public Mode getMode() {
    return currMode;
  }

  // Returns the current position of the left arm in shaft rotations.
  public double getLeftArmPosition() {
    return leftIntakeDeploy.getPosition().getValueAsDouble();
  }

  // Returns the desired position for the left arm.
  public double getLeftArmDesiredPosition() {
    return desiredLeftArmPosition;
  }

  // Returns the current velocity of the left roller in rotations per second.
  public double getLeftRollerVelocity() {
    return leftIntake.getVelocity().getValueAsDouble();
  }

  // Returns the desired velocity for the left roller.
  public double getLeftRollerDesiredVelocity() {
    return desiredLeftRollerVelocity;
  }

  // Returns true if the left arm is within 0.5 units of the desired position.
  public boolean leftArmInPosition() {
    return Math.abs(getLeftArmPosition() - desiredLeftArmPosition) < 0.5;
  }

  // Returns true if the left roller is within 1.0 units of the desired velocity.
  public boolean leftRollerAtSpeed() {
    return Math.abs(getLeftRollerVelocity() - desiredLeftRollerVelocity) < 1.0;
  }

  // Returns the current position of the right arm in shaft rotations.
  public double getRightArmPosition() {
    return rightIntakeDeploy.getPosition().getValueAsDouble();
  }

  // Returns the desired position for the right arm.
  public double getRightArmDesiredPosition() {
    return desiredRightArmPosition;
  }

  // Returns the current velocity of the right roller in rotations per second.
  public double getRightRollerVelocity() {
    return rightIntake.getVelocity().getValueAsDouble();
  }

  // Returns the desired velocity for the right roller. 
  public double getRightRollerDesiredVelocity() {
    return desiredRightRollerVelocity;
  }
  
  // Returns true if the right arm is within 0.5 units of the desired position.
  public boolean rightArmInPosition() {
    return Math.abs(getRightArmPosition() - desiredRightArmPosition) < 0.5;
  }

  // Returns true if the right roller is within 1.0 units of the desired velocity.
  public boolean rightRollerAtSpeed() {
    return Math.abs(getRightRollerVelocity() - desiredRightRollerVelocity) < 1.0;
  }

  // Returns true if both arms are in position and both rollers are at speed.
  public boolean isReady() {
    return leftArmInPosition() && leftRollerAtSpeed() && rightArmInPosition() && rightRollerAtSpeed();
  }

  public void updateDash() {
    SmartDashboard.putString("Intake Mode", currMode.toString());
    SmartDashboard.putBoolean("Intake Ready", isReady());
    SmartDashboard.putNumber("Left Arm Position", getLeftArmPosition());
    SmartDashboard.putNumber("Left Arm Desired", getLeftArmDesiredPosition());
    SmartDashboard.putNumber("Left Roller Vel", getLeftRollerVelocity());
    SmartDashboard.putNumber("Right Arm Position", getRightArmPosition());
    SmartDashboard.putNumber("Right Arm Desired", getRightArmDesiredPosition());
    SmartDashboard.putNumber("Right Roller Vel", getRightRollerVelocity());
  }
  
  private void configMotor(TalonFX motor, boolean invert, double currentLimit, boolean isArmMotor) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
    
    if (isArmMotor) {
      // Position control for arm motors
      motorConfigs.Slot0.kP = 2.0;
      motorConfigs.Slot0.kI = 0.5;
      motorConfigs.Slot0.kD = 0.0;
    } else {
      // Velocity control for roller motors
      motorConfigs.Slot0.kP = 0.2;
      motorConfigs.Slot0.kI = 0.1;
      motorConfigs.Slot0.kD = 0.0;
      motorConfigs.Slot0.kV = 0.12;
      motorConfigs.Slot0.kS = 0.1;
    }
    
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}
