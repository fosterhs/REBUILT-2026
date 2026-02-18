package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
  private final TalonFX rightIntakeDeploy = new TalonFX(14, canivore);
  private final TalonFX rightIntake = new TalonFX(15, canivore);
  private final TalonFX leftIntakeDeploy = new TalonFX(16, canivore);
  private final TalonFX leftIntake = new TalonFX(17, canivore);
  //mr shaman said not using the sensor no more...IDK TALK TO HIM🤷‍♀️
  
  private final Timer leftIntakeTimer = new Timer();
  private final Timer rightIntakeTimer = new Timer();
  private boolean isHomed = false;
  // Code Review: Put more requests per motor stated  - here👇, do I look like chatgpt?🥀 so demanding..."pUt MoRe ReQuEsT"(mocking)
  private final PositionVoltage leftArmPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage rightArmPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage leftRollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage rightRollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut leftArmVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightArmVoltageRequest = new VoltageOut(0.0);
  
  private final StatusSignal<AngularVelocity> leftArmVelocity;
  private final StatusSignal<AngularVelocity> rightArmVelocity;
  private final StatusSignal<Angle> leftArmEncoderPosition;
  private final StatusSignal<Angle> rightArmEncoderPosition;

  public Intake() {
    configMotor(rightIntakeDeploy, true, 40.0, true);
    configMotor(rightIntake, false, 60.0, false);
    configMotor(leftIntakeDeploy, false, 40.0, true);
    configMotor(leftIntake, true, 60.0, false);
    
    leftArmVelocity = leftIntakeDeploy.getVelocity();
    rightArmVelocity = rightIntakeDeploy.getVelocity();
    leftArmEncoderPosition = leftIntakeDeploy.getPosition();
    rightArmEncoderPosition = rightIntakeDeploy.getPosition();
    
    rightIntakeTimer.start();
    leftIntakeTimer.start();
  }

  public void init() {
    if (currMode != Mode.HOME) {//if not at mode.home, set mode and arm positions at 0
      currMode = Mode.HOME;
      desiredLeftArmPosition = 0.0;
      desiredLeftRollerVelocity = 0.0;
      desiredRightArmPosition = 0.0;
      desiredRightRollerVelocity = 0.0;
      isHomed = false;
    }
    leftIntakeTimer.restart();
    rightIntakeTimer.restart();
  }

  public void periodic() {
    leftArmVelocity.refresh();
    rightArmVelocity.refresh();
    leftArmEncoderPosition.refresh();
    rightArmEncoderPosition.refresh();
    
    switch (currMode) {
      case HOME:
        leftIntakeDeploy.setControl(leftArmVoltageRequest.withOutput(-2.0));
        rightIntakeDeploy.setControl(rightArmVoltageRequest.withOutput(-2.0));
        leftIntake.setControl(leftRollerVelocityRequest.withVelocity(0.0));
        rightIntake.setControl(rightRollerVelocityRequest.withVelocity(0.0));
        if (Math.abs(leftArmVelocity.getValueAsDouble()) > 0.05) {// Code Review: Puting left and right together just put separately -...ts grammar bruh, timer breakdown👇
          leftIntakeTimer.restart();
        }
        if (Math.abs(rightArmVelocity.getValueAsDouble()) > 0.05) {
          rightIntakeTimer.restart();
        }

        if (leftIntakeTimer.get() >1.0 && rightIntakeTimer.get() > 1.0) {
          leftIntakeDeploy.setPosition(0.0, 0.03);
          rightIntakeDeploy.setPosition(0.0, 0.03);
          isHomed = true;
          currMode = Mode.STOW;
          leftIntakeTimer.restart();
          rightIntakeTimer.restart();
        }
        break;

      case LEFT:
        if (rightArmEncoderPosition.getValueAsDouble() < 0.67) {
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = 2.0;
          desiredLeftRollerVelocity = 10.0;
          desiredRightRollerVelocity = 0.0;
        } else {
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = leftArmEncoderPosition.getValueAsDouble();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0;
          //intakeTimer.restart(); // Code review: WHAT POINT - FINE REMOVED 👇GEE YELLING IN ALL CAPS?
        }
        break;

      case RIGHT:
        if (leftArmEncoderPosition.getValueAsDouble() < 0.67) {
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = 2.0;
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 10.0;
        } else {
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = rightArmEncoderPosition.getValueAsDouble();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0; 
        }
        break;
      case STOW: // Code Review: Dont use this, go to zero position directly. From line 115-135 - bruh 🫱💻🫲 here, write the code bro
        desiredLeftArmPosition = 0.0;
        desiredRightArmPosition = 0.0;
        desiredLeftRollerVelocity = 0.0;
        desiredRightRollerVelocity = 0.0;
        break;
    }
    if (currMode != Mode.HOME) {
      leftIntakeDeploy.setControl(leftArmPositionRequest.withPosition(desiredLeftArmPosition));
      leftIntake.setControl(leftRollerVelocityRequest.withVelocity(desiredLeftRollerVelocity));
      rightIntakeDeploy.setControl(rightArmPositionRequest.withPosition(desiredRightArmPosition));
      rightIntake.setControl(rightRollerVelocityRequest.withVelocity(desiredRightRollerVelocity));
    }
  }

  public void leftIntake() {
    if (isHomed) {
      currMode = Mode.LEFT;
      leftIntakeTimer.restart();
      rightIntakeTimer.restart();
    }
  }

  public void rightIntake() {
    if (isHomed) {
      currMode = Mode.RIGHT;
      leftIntakeTimer.restart();
      rightIntakeTimer.restart();
    }
  }

  public void stowIntake() {
    if (isHomed) {
      currMode = Mode.STOW;
      leftIntakeTimer.restart();
      rightIntakeTimer.restart();
    }
  }

  public Mode getMode() { 
    return currMode;
  }

  public double getLeftArmEncoderPosition() {
    return leftArmEncoderPosition.getValueAsDouble();
  }

  public double getLeftArmDesiredPosition() {
    return desiredLeftArmPosition;
  }

  public double getLeftRollerVelocity() {
    return leftIntake.getVelocity().getValueAsDouble();
  }

  public double getLeftRollerDesiredVelocity() {
    return desiredLeftRollerVelocity;
  }

  public boolean leftArmInPosition() {
    return Math.abs(getLeftArmEncoderPosition() - desiredLeftArmPosition) < 0.1;
  }

  public boolean leftRollerAtSpeed() {
    return Math.abs(getLeftRollerVelocity() - desiredLeftRollerVelocity) < 1.0;
  }

  public double getRightArmEncoderPosition() {
    return rightArmEncoderPosition.getValueAsDouble();
  }

  public double getRightArmDesiredPosition() {
    return desiredRightArmPosition;
  }

  public double getRightRollerVelocity() {
    return rightIntake.getVelocity().getValueAsDouble();
  }

  public double getRightRollerDesiredVelocity() {
    return desiredRightRollerVelocity;
  }
  
  public boolean rightArmInPosition() {
    return Math.abs(getRightArmEncoderPosition() - desiredRightArmPosition) < 0.1;
  }

  public boolean rightRollerAtSpeed() {
    return Math.abs(getRightRollerVelocity() - desiredRightRollerVelocity) < 1.0;
  }

  public boolean isReady() {
    return leftArmInPosition() && leftRollerAtSpeed() && 
           rightArmInPosition() && rightRollerAtSpeed();
  }

  public boolean getIsHomed() {
    return isHomed;
  }

  public void updateDash() {
    SmartDashboard.putString("Intake Mode", currMode.toString());
    SmartDashboard.putBoolean("Intake Ready", isReady());
    SmartDashboard.putBoolean("Intake Homed", isHomed);
    SmartDashboard.putNumber("Left Intake Timer", leftIntakeTimer.get());
    SmartDashboard.putNumber("Right Intake Timer", rightIntakeTimer.get());
    SmartDashboard.putNumber("Left Arm Encoder", getLeftArmEncoderPosition());
    SmartDashboard.putNumber("Left Arm Desired", getLeftArmDesiredPosition());
    SmartDashboard.putNumber("Left Roller Vel", getLeftRollerVelocity());
    SmartDashboard.putNumber("Right Arm Encoder", getRightArmEncoderPosition());
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
      motorConfigs.Slot0.kP = 2.0;
      motorConfigs.Slot0.kI = 0.5;
      motorConfigs.Slot0.kD = 0.0;
    } else {
      motorConfigs.Slot0.kP = 0.2;
      motorConfigs.Slot0.kI = 0.1;
      motorConfigs.Slot0.kD = 0.0;
      motorConfigs.Slot0.kV = 0.12;
      motorConfigs.Slot0.kS = 0.1;
    }
    
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}
//add another system where if sensor reads range too far and arm intake both distance too far, try to set arm back to position 0 and stop everything. (shaman said not doing it)
