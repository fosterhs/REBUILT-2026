package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;

public class Indexer {
  public enum Mode {FORWARD, JAM, IDLE}
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hopperIndexMotor = new TalonFX(19, canivore);
  private final TalonFX shooterIndexMotor = new TalonFX(10, canivore);
  private final CANrange shooterSensor = new CANrange(11, canivore);
  private final CANrange hopperSensor = new CANrange(12, canivore);
  private final VoltageOut hopperIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut shooterIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final StatusSignal<Boolean> shooterFuelDetected;
  private final StatusSignal<Boolean> hopperFuelDetected;
  private final Timer shooterTimer = new Timer();
  private final Timer hopperTimer = new Timer();
  private final Timer jamTimer = new Timer();
  private Mode currMode = Mode.IDLE;
  
  // Initialize indexer: configure motor, start timer, configure sensor, and obtain fuelDetected data
  public Indexer() {
    configIndexMotor(hopperIndexMotor, false);
    configIndexMotor(shooterIndexMotor,false);
    configCANrange(shooterSensor, 0.45);
    configCANrange(hopperSensor, 0.45);
    shooterFuelDetected = shooterSensor.getIsDetected();
    hopperFuelDetected = hopperSensor.getIsDetected();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterFuelDetected, hopperFuelDetected);
	  ParentDevice.optimizeBusUtilizationForAll(hopperIndexMotor, shooterIndexMotor, shooterSensor, hopperSensor);
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
    if (currMode != Mode.JAM) {
      jamTimer.restart();
    }

    switch (currMode) {
      case FORWARD://just going forward
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(12.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(12.0).withEnableFOC(true));
        if (hopperTimer.get() > 3.0) {
          currMode = Mode.IDLE;
        }
        else if (shooterTimer.get() > 5.0 && hopperTimer.get() < 3.0) {
          currMode = Mode.JAM;
          jamTimer.restart();
        }
      break;

      case JAM://execute when sensor is on false for 5 second
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(-12.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(-12.0).withEnableFOC(true));
        if (hopperTimer.get() > 3.0) {
          currMode = Mode.IDLE;
        }
        else if (shooterTimer.get() < 5.0 || getJamTimer() > 0.7) {
          currMode = Mode.FORWARD;
        }
      break;

      case IDLE://just stops
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
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

  // return a boolean if the sensor sense a fuel.
  public boolean getShooterSensor() {
    return shooterFuelDetected.getValue();
  }

  public boolean getHopperSensor() {
    return hopperFuelDetected.getValue();
  }

  // Publish indexer information (state, sensor, shooting flag, and timer) to SmartDashboard
  public void updateDash() {
    //SmartDashboard.putString("Indexer getMode", getMode().toString());
    //SmartDashboard.putBoolean("Indexer getShooterSensor", getShooterSensor());
    //SmartDashboard.putNumber("Indexer getShooterTimer", getShooterTimer());
    //SmartDashboard.putBoolean("Indexer getHopperSensor", getHopperSensor());
    //SmartDashboard.putNumber("Indexer getHopperTimer", getHopperTimer());
    //SmartDashboard.putNumber("Indexer getJammedTimer", getJamTimer());
  }
  
  // Configs the motor settings and PID
  private void configIndexMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  // Configs the sensor settings 
  private void configCANrange(CANrange sensor, double threshold) {
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();

    sensorConfigs.ProximityParams.ProximityThreshold = threshold;

    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
}