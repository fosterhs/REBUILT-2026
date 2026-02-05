package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
private final CANBus canivore = new CANBus("canivore");
private final TalonFX shootMotor = new TalonFX(10, canivore);  // Initializes the shootMotor
private final StatusSignal<AngularVelocity> shooterVelocity;
private final VelocityVoltage shooterMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
private final double RPMtol = 20.0;
private double desiredRPM = 0.0;
private boolean isSpunUp = false;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configMotor(shootMotor, false); // Configures the motor with counterclockwise rotation positive.
    shooterVelocity = shootMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocity);
	  ParentDevice.optimizeBusUtilizationForAll(shootMotor);
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