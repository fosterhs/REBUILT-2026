package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
  public enum Mode {FORWARD, JAM, IDLE}
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX indexMotor = new TalonFX(0, canivore);
  private final VelocityVoltage indexMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final CANrange shooterSensor = new CANrange(11, canivore);
  private final CANrange hopperSensor = new CANrange(12, canivore);
  private final Timer shooterTimer = new Timer();
  private final Timer hopperTimer = new Timer();
  private final Timer jamTimer = new Timer();
  private final StatusSignal<Boolean> shooterFuelDetected;
  private final StatusSignal<Boolean> hopperFuelDetected;
  private Mode currMode = Mode.IDLE;
  
  // Initialize indexer: configure motor, start timer, configure sensor, and obtain fuelDetected data
  public Indexer() {
    configMotor(indexMotor);
    configCANrange(shooterSensor);
    configCANrange(hopperSensor);
    shooterFuelDetected = shooterSensor.getIsDetected();
    hopperFuelDetected = hopperSensor.getIsDetected();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterFuelDetected, hopperFuelDetected);
	  ParentDevice.optimizeBusUtilizationForAll(indexMotor, shooterSensor, hopperSensor);
    shooterTimer.restart();
    hopperTimer.restart();
    jamTimer.restart();
  }

  // Runs code once at start: set current state to IDLE, stop shooting, stops motor, and reset the index timer
  public void init() {
    currMode = Mode.IDLE;
  }

  // Runs code periodically: refresh the fuel sensor, run the shooting/jam state machine, and set motor outputs for each state
  public void periodic() {
    shooterFuelDetected.refresh();
    hopperFuelDetected.refresh();

    if (getShooterSensor() || currMode == Mode.IDLE) {
     shooterTimer.restart();
    }
    if (getHopperSensor()) {
      hopperTimer.restart();
    }
    if (currMode == Mode.IDLE || currMode == Mode.FORWARD) {
      jamTimer.restart();
    }

    switch (currMode) {
      case FORWARD://just going forward
        indexMotor.setControl(indexMotorVelocityRequest.withVelocity(100.0).withEnableFOC(true));
        if (hopperTimer.get() > 3.0) {
          currMode = Mode.IDLE;
        }
        else if (shooterTimer.get() > 5.0 && hopperTimer.get() < 3.0) {
          currMode = Mode.JAM;
          jamTimer.restart();
        }
      break;

      case JAM://execute when sensor is on false for 5 second
        indexMotor.setControl(indexMotorVelocityRequest.withVelocity(-100.0).withEnableFOC(true));
        if (hopperTimer.get() > 3.0) {
          currMode = Mode.IDLE;
        }
        else if (shooterTimer.get() < 5.0 || getJamTimer() > 0.7) {
          currMode = Mode.FORWARD;
        }
      break;

      case IDLE://just stops
        indexMotor.setControl(indexMotorVelocityRequest.withVelocity(0.0).withEnableFOC(true));
      break;
    }
  }

  // Marks the Indexer as running forward (not shooting) and resets the jam timer.
  public void start() {
    currMode = Mode.FORWARD;
  }

  //Marks the indexer as idle, stops shooting, and reset the jam timer.
  public void stop() {
    currMode = Mode.IDLE;
  }
  
  // Mark the indexer as jammed, stop shooting, and reset the timer used for jam handling.
  public void jammed() {
    currMode = Mode.JAM;
  }

  // Returns the current mode that the indexer is in.
  public Mode getMode() {
    return currMode;
  }

  // Return elapsed time (seconds) from IndexTimer since last reset
  public double getShooterTimer() {
    return shooterTimer.get();
  }

  public double getHopperTimer() {
    return hopperTimer.get();
  }

  public double getJamTimer() {
    return jamTimer.get();
  }

  // return a boolean if the shooter sensor sense a fuel.
  public boolean getShooterSensor() {
    return shooterFuelDetected.getValue();
  }

  // return a boolean if the hopper sensor sense a fuel.
  public boolean getHopperSensor() {
    return hopperFuelDetected.getValue();
  }

  // Publish indexer information (state, sensor, shooting flag, and timer) to SmartDashboard
  public void updateDash() {
    SmartDashboard.putString("Indexer State", currMode.toString());
    SmartDashboard.putBoolean("shooter sensor", getShooterSensor());
    SmartDashboard.putNumber("shooter Sensor Timer", getShooterTimer());
    SmartDashboard.putBoolean("Hopper sensor", getHopperSensor());
    SmartDashboard.putNumber("Hopper Sensor Timer", getHopperTimer());
    SmartDashboard.putNumber("Jammed Timer", getJamTimer());
  }
  
  // Configs the motor settings and PID
  private void configMotor(TalonFX motor) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfigs.Slot0.kP = 0.2;
    motorConfigs.Slot0.kI = 0.1;
    motorConfigs.Slot0.kD = 0.0;
    motorConfigs.Slot0.kV = 0.12;
    motorConfigs.Slot0.kS = 0.1;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  // Configs the sensor settings 
  private void configCANrange(CANrange sensor) {
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();

    sensorConfigs.ProximityParams.ProximityThreshold = 0.45;

    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
}