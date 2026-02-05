package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
  public enum Mode {HOME, LEFT, RIGHT, STOW} //
  public Mode currMode = Mode.HOME; //by default, the intake starts in the home position

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX indexLeftRollerMotor = new TalonFX(0, canivore);
  private final TalonFX indexRightRollerMotor = new TalonFX(1, canivore);
  private final TalonFX intakeLeftArmMotor  = new TalonFX(2, canivore);
  private final TalonFX intakeRightArmMotor = new TalonFX(3, canivore);

  private final VelocityVoltage indexMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  public double desiredLeftArmPosition = 0.0; //
  public double desiredLeftRollerVelocity = 0.0; //
  public double desiredRightArmPosition = 0.0; //
  public double desiredRightRollerVelocity = 0.0; //

  //
  public Intake() {
    configMotor(indexLeftRollerMotor, false);
    configMotor(indexRightRollerMotor, false);
    configMotor(intakeLeftArmMotor, false);
    configMotor(intakeRightArmMotor, false);
  }

  //
  public void init() {
    currMode = Mode.HOME;
  }

  //
  public void periodic() {
    switch (currMode) {
      case HOME:

      break;

      case LEFT:

      break;

      case RIGHT:

      break;

      case STOW:
        
      break;
    }
  }

  //
  public void leftIntake() {

  }

  //
  public void rightIntake() {

  }

  //
  public void stowIntake() {

  }

  //
  public Mode getMode() {
    return currMode;
  }

  //
  public double getLeftArmPosition() {
    return 0.0;
  }

  //
  public double getLeftArmDesiredPosition() {
    return 0.0;
  }

  //
  public double getLeftRollerVelocity() {
    return 0.0;
  }

  //
  public double getLeftRollerDesiredVelocity() {
    return 0.0;
  }

  //
  public boolean leftArmInPosition() {
    return true;
  }

  //
  public boolean leftRollerAtSpeed() {
    return true;
  }

  //
  public double getRightArmPosition() {
    return 0.0;
  }

  //
  public double getRightArmDesiredPosition() {
    return 0.0;
  }

  //
  public double getRightRollerVelocity() {
    return 0.0;
  }

  //
  public double getRightRollerDesiredVelocity() {
    return 0.0;
  }
  
  //
  public boolean rightArmInPosition() {
    return true;
  }

  //
  public boolean rightRollerAtSpeed() {
    return true;
  }

  // Returns true or false based on whether the left arm, left roller, right arm, and right roller are all in their correct positions/velocities.
  public boolean isReady() {
    return leftArmInPosition() && leftRollerAtSpeed() && rightArmInPosition() && rightRollerAtSpeed();
  }

  //
  public void updateDash() {
    SmartDashboard.putString("Intake State", currMode.toString());
  }

  // Configs the motor settings and PID
  private void configMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}