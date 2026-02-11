package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
private final CANBus canivore = new CANBus("canivore");
private final TalonFX shootMotor = new TalonFX(11, canivore);  // Initializes the shootMotor
private final TalonFX hoodMotor = new TalonFX(9, canivore);
private final CANcoder hoodEncoder = new CANcoder(28);
private final StatusSignal<AngularVelocity> shooterVelocity;
private final MotionMagicVelocityVoltage shooterMotorVelocityRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
private final double RPMtol = 180.0;
private final MotionMagicTorqueCurrentFOC hoodMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0);
private final StatusSignal<Angle> hoodPosition;
private final double hoodGearRatio = 294.0/20.0;
private final double hoodMin = 0.02;
private final double hoodMax = 0.12;
private double desiredRPM = 0.0;
private boolean isSpunUp = false;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configShootMotor(shootMotor, false); // Configures the motor with counterclockwise rotation positive.
    configHoodMotor(hoodMotor, false);
    configHoodEncoder(hoodEncoder);
    hoodPosition = hoodMotor.getPosition();
    shooterVelocity = shootMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocity, hoodPosition);
	  ParentDevice.optimizeBusUtilizationForAll(shootMotor, hoodMotor, hoodEncoder);
  }

  // Turns on motor. Sets the spped of the motor in rotations per minute.
  public void spinUp(double rpm) {
    setRPM(rpm);
    desiredRPM = rpm;
    isSpunUp = true;
  }

  // Turn off motor.
  public void spinDown() {
    setRPM(0.0);
    desiredRPM = 0.0;
    isSpunUp = false;
  }

  public void setHoodPosition(double desiredPosition) {
    if (desiredPosition < hoodMin) desiredPosition = hoodMin;
    if (desiredPosition > hoodMax) desiredPosition = hoodMax;
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(desiredPosition));
  }

  // Returns true or false based on whether the shooter motor is turned on. Does not indicate whether the motor has reached the desired RPM.
  public boolean isSpunUp() {
    return isSpunUp;
  }

  // Returns true or false based on whether the shooter motor is near the desired RPM.
  public boolean isAtSpeed() {
    return Math.abs(desiredRPM - getRPM()) < RPMtol;
  }

  // Returns the motor velocity as a double in RPS (Rotations Per Second)
  public double getRPS() {
    return shooterVelocity.refresh().getValueAsDouble();
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getRPM() {
    return getRPS() * 60.0;
  }

  // Sets the spped of the motor in rotations per minute.
  private void setRPM(double rpm) {
    double rps = rpm / 60.0;
    shootMotor.setControl(shooterMotorVelocityRequest.withVelocity(rps).withEnableFOC(true));
  }

  // Publish Shooter information (Motor state, Velocity) to SmartDashboard.
  public void updateDash() {
    SmartDashboard.putNumber("Shooter desiredRPM", desiredRPM);
    SmartDashboard.putBoolean("Shooter isAtSpeed", isAtSpeed());
    SmartDashboard.putBoolean("Shooter isSpunUp", isSpunUp());
    SmartDashboard.putNumber("Shooter Velocity (RPS)", getRPS());
    SmartDashboard.putNumber("Shooter Velocity (RPM)", getRPM());
  }

  // Configures the swerve module's drive motor.
  private void configShootMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.33; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.12; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.002; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.1; // The amount of voltage required to barely overcome static friction in the swerve wheel.

    motorConfigs.MotionMagic.MotionMagicAcceleration = 200.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 1000.0;


    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Configures the swerve module's turn motor.
  private void configHoodMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*hoodGearRatio); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*hoodGearRatio); // Units: roations per second.

    // CANcoder feedback configurations.
    motorConfigs.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = hoodGearRatio;
    motorConfigs.ClosedLoopGeneral.ContinuousWrap = false;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Configures the swerve module's wheel encoder.
  private void configHoodEncoder(CANcoder encoder) {
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();

    encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfigs.MagnetSensor.MagnetOffset = 0.08;

    encoder.getConfigurator().apply(encoderConfigs, 0.03);
  }
}