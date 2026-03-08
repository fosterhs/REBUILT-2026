package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class Shooter {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hoodMotor = new TalonFX(9, canivore);  
  private final TalonFX shootMotorRight = new TalonFX(12, canivore);  
  private final TalonFX shootMotorLeft = new TalonFX(11, canivore); 
  private final CANcoder hoodEncoder = new CANcoder(28, canivore); 
  private final StatusSignal<AngularVelocity> shooterVelocityRight;
  private final StatusSignal<AngularVelocity> shooterVelocityLeft;
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Voltage> shooterVoltageRight;
  private final VelocityVoltage shooterMotorRightVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final Follower shooterMotorLeftFollowerRequest = new Follower(12, MotorAlignmentValue.Opposed);
  private final MotionMagicTorqueCurrentFOC hoodMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); 
  private final double rpmTol = 200.0; // Can adjust
  private final double hoodTol = 0.010; // Can adjust
  public final double hoodMinPosition = 0.020; // Can adjust
  public final double hoodMaxPosition = 0.115; // Can adjust
  private final Timer shooterAtSpeedTimer = new Timer(); // Timer to track how long the shooter has been at speed. Used to prevent the shooter from being considered ready if it is only briefly at speed.
  private final Timer shooterNotAtSpeedTimer = new Timer(); // Timer to track how long the shooter has not been at speed. Used to prevent the shooter from being considered ready if it is only briefly at speed.
  private final double shooterDelay = 0.5; // Seconds that the shooter must be at speed before it is considered ready. Can adjust.
  private boolean flywheelIsReady = false; // Whether the shooter is at speed long enough to be considered ready. Updated periodically based on the shooterAtSpeedTimer and shooterNotAtSpeedTimer.
  private boolean flywheelIsAtSpeed = false; // Whether the shooter is currently at speed. Updated periodically in periodic().
  private double shootingRPM = 3000.0; // Can adjust
  private double desiredHoodPosition = hoodMinPosition;

  // Simulation
  private final TalonFXSimState hoodMotorSim = hoodMotor.getSimState();
  private final TalonFXSimState shootMotorRightSim = shootMotorRight.getSimState();
  private final TalonFXSimState shootMotorLeftSim = shootMotorLeft.getSimState();
  private final CANcoderSimState hoodEncoderSim = hoodEncoder.getSimState();
  private double desiredRPMSim = 0;
  public static final double hoodGearRatio = 1;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configHoodEncoder(hoodEncoder);
    configShootMotor(shootMotorRight, true);
    configShootMotor(shootMotorLeft, false); // Configures the motor with counterclockwise rotation positive.
    configHoodMotor(hoodMotor, false);
    shooterVelocityRight = shootMotorRight.getVelocity();
    shooterVelocityLeft = shootMotorLeft.getVelocity();
    hoodPosition = hoodEncoder.getAbsolutePosition();
    shooterVoltageRight = shootMotorRight.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocityRight, shooterVelocityLeft, hoodPosition, shooterVoltageRight);
	  ParentDevice.optimizeBusUtilizationForAll(shootMotorLeft, hoodMotor, hoodEncoder);
  }

  public void init() {
    shooterAtSpeedTimer.restart();
    shooterNotAtSpeedTimer.restart();
    spinDown();
    lowerHood();
    flywheelIsReady = false;
  }
  
  public void periodic() {
    flywheelIsAtSpeed = Math.abs(shootingRPM - getLeftFlywheelMotorRPM()) < rpmTol && Math.abs(shootingRPM - getRightFlywheelMotorRPM()) < rpmTol;
    if (!flywheelIsAtSpeed) shooterAtSpeedTimer.restart();
    if (flywheelIsAtSpeed) shooterNotAtSpeedTimer.restart();

    if (shooterAtSpeedTimer.get() > shooterDelay && !flywheelIsReady) flywheelIsReady = true;
    if (shooterNotAtSpeedTimer.get() > shooterDelay && flywheelIsReady) flywheelIsReady = false;
  }
  
  // Turns on motor. Sets the speed of the motor in rotations per minute.
  public void spinUp() {
    desiredRPMSim = shootingRPM;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(shootingRPM/60.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftFollowerRequest);
  }

  // Turn off motor.
  public void spinDown() {
    desiredRPMSim = 0;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(0.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftFollowerRequest);
  }

  public void setFlywheelRPM(double rpm) {
    if (rpm > 5000.0) {
      shootingRPM = 5000.0;
    } else if (rpm < 600.0) {
      shootingRPM = 600.0;
    } else {
      shootingRPM = rpm;
    }
  }

  public void setHoodPosition(double position) {
    if (position < hoodMinPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
      desiredHoodPosition = hoodMinPosition;
    } else if (position > hoodMaxPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMaxPosition));
      desiredHoodPosition = hoodMaxPosition;
    } else {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(position));
      desiredHoodPosition = position;
    }
  }

  public void lowerHood() {
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
    desiredHoodPosition = hoodMinPosition;
  }

  public boolean hoodIsInPosition() {
    // TODO: Fix simulationPeriodic
    if (Robot.isSimulation()) return true;

    return Math.abs(desiredHoodPosition - getHoodPosition()) < hoodTol;
  }

  public double getHoodPosition() {
    return hoodPosition.refresh().getValueAsDouble();
  }

  // Returns true or false based on whether the shooter motor is near the desired RPM.
  public boolean flywheelIsReady() {
    return flywheelIsReady;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getLeftFlywheelMotorRPM() {
    return shooterVelocityLeft.refresh().getValueAsDouble()*60.0;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getRightFlywheelMotorRPM() {
    return shooterVelocityRight.refresh().getValueAsDouble()*60.0;
  }

  public boolean isReady() {
    return flywheelIsReady() && hoodIsInPosition();
  }

  // Publish Shooter information (Motor state, Velocity) to SmartDashboard.
  public void updateDash() {
    // SmartDashboard.putNumber("Shooter getRightShooterRPM", getRightShooterRPM());
    // SmartDashboard.putNumber("Shooter getLeftShooterRPM", getLeftShooterRPM());
    // SmartDashboard.putBoolean("Shooter shooterIsAtSpeed", shooterIsAtSpeed());
    // SmartDashboard.putNumber("Shooter shootingRPM", shootingRPM);
    // SmartDashboard.putNumber("Shooter getHoodPosition", getHoodPosition());
    // SmartDashboard.putBoolean("Shooter hoodIsInPosition", hoodIsInPosition());
    // SmartDashboard.putNumber("Shooter desiredHoodPosition", desiredHoodPosition);
    // SmartDashboard.putBoolean("Shooter isReady", isReady());
  }

  public void simulationPeriodic() {
    // Very basic - just jump to the desired values
    // Update the motors
    shootMotorRightSim.setRotorVelocity(desiredRPMSim / 60.0);
    shootMotorLeftSim.setRotorVelocity(desiredRPMSim / 60.0);

    // Update the hood position.
    // TODO:
    hoodMotorSim.setRawRotorPosition(desiredHoodPosition/hoodGearRatio);
    hoodEncoderSim.setRawPosition(desiredHoodPosition/hoodGearRatio);
  }

  private void configHoodEncoder(CANcoder CANsensor) {
    CANcoderConfiguration sensorConfigs = new CANcoderConfiguration();

    sensorConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; 
    sensorConfigs.MagnetSensor.MagnetOffset = 0.378662109375;
    sensorConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    CANsensor.getConfigurator().apply(sensorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configShootMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.20; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.02; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120.0;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configHoodMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorConfigs.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = 211.68;

    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 30.0;

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*211.68); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*211.68); // Units: roations per second.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}