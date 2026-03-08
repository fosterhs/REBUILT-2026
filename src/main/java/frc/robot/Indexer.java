package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.CANBus;

public class Indexer {
  public enum Mode {INDEX, IDLE}
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hopperIndexMotor = new TalonFX(19, canivore);
  private final TalonFX shooterIndexMotor = new TalonFX(10, canivore);
  private final VoltageOut hopperIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut shooterIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private Mode currMode = Mode.IDLE;
  private final Timer indexTimer = new Timer();
  private double indexVoltage = 12.0;

  // Simulation
  private final TalonFXSimState hopperIndexMotorSim = hopperIndexMotor.getSimState();
  private final TalonFXSimState shooterIndexMotorSim = shooterIndexMotor.getSimState();
  private boolean hopperSimHasFuel = false;
  
  // Initialize indexer: configure motor, start timer, configure sensor, and obtain fuelDetected data
  public Indexer() {
    configIndexMotor(hopperIndexMotor, false);
    configIndexMotor(shooterIndexMotor,true);
	  ParentDevice.optimizeBusUtilizationForAll(hopperIndexMotor, shooterIndexMotor);
    indexTimer.restart();
  }

  // Runs code once at start: set current state to IDLE, stop shooting, stops motor, and reset the index timer
  public void init() {
    currMode = Mode.IDLE;
    indexTimer.restart();
  }

  // Runs code periodically: refresh the fuel sensor, run the shooting/jam state machine, and set motor outputs for each state
  public void periodic() {
    switch (currMode) {
      case INDEX://just going forward
        if (indexTimer.get() < 1.7) {
          hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(indexVoltage).withEnableFOC(true));
          shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(indexVoltage).withEnableFOC(true));
        } else if (indexTimer.get() < 2.0) {
          hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(-indexVoltage).withEnableFOC(true));
          shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(-indexVoltage).withEnableFOC(true));
        } else {
          indexTimer.restart();
        }
      break;

      case IDLE://just stops
        indexTimer.restart();
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      break;
    }
  }

  public void setIndexVoltage(double voltage) {
    indexVoltage = voltage;
  }

  // Marks the Indexer as running forward (not shooting) and resets the jam timer.
  public void start() {
    currMode = Mode.INDEX;
  }

  //Marks the indexer as idle, stops shooting, and reset the jam timer.
  public void stop() {
    currMode = Mode.IDLE;
  }

  // Returns the current mode that the indexer is in.
  public Mode getMode() {
    return currMode;
  }

  // Publish indexer information (state, sensor, shooting flag, and timer) to SmartDashboard
  public void updateDash() {
    //SmartDashboard.putString("Indexer getMode", getMode().toString());
  }

  public void simulationPeriodic() {
    
  }
  
  // Configs the motor settings and PID
  private void configIndexMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 30.0;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}