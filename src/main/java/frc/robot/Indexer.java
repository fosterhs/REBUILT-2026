package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.CANBus;

public class Indexer {
  public enum Mode {INDEX, IDLE} // INDEX mode runs the indexer motors to feed fuel into the shooter. IDLE mode stops the motors.
  private final CANBus canivore = new CANBus("canivore"); // Creates a new CAN bus called "canivore". This is the name of the CAN bus that the indexer motors are connected to. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX hopperIndexMotor = new TalonFX(19, canivore); // Creates a new TalonFX motor controller for the hopper indexer motor. The motor is assigned an ID of 19 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX shooterIndexMotor = new TalonFX(10, canivore); // Creates a new TalonFX motor controller for the shooter indexer motor. The motor is assigned an ID of 10 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final VoltageOut hopperIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the hopper indexer motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final VelocityVoltage shooterIndexMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true); // Creates a new VoltageOut control mode for the shooter indexer motor. This will allow us to set the voltage that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final Timer indexTimer = new Timer(); // Creates a new timer that we will use to time how long the indexer has been running. This will allow us to run the motors for a specific amount of time before stopping or reversing them.
  private Mode currMode = Mode.IDLE; // Initializes the current mode of the indexer to IDLE. This means that when the robot is first turned on, the indexer will be in the IDLE state and will not be running.
  private double indexRPM = 4000.0; // Initializes the voltage that the indexer motors will run at when the indexer is running. This can be adjusted.
  private double indexVoltage = 12.0;
  private boolean isSpoolingUp = false; 

  // Simulation
  private final TalonFXSimState hopperIndexMotorSim = hopperIndexMotor.getSimState();
  private final TalonFXSimState shooterIndexMotorSim = shooterIndexMotor.getSimState();
  private boolean hopperSimHasFuel = false;
  
  // Constructor for the Indexer class. This is where we will configure the indexer motors with the appropriate settings for our robot. We will set the neutral mode to brake, set the motor direction based on the invert parameter, and configure current limits for the motor. We will also optimize bus utilization for the motors to improve performance.
  public Indexer() {
    configIndexMotor(hopperIndexMotor, false);
    configIndexMotor(shooterIndexMotor,true);
    ParentDevice.optimizeBusUtilizationForAll(hopperIndexMotor, shooterIndexMotor);
  }

  // Resets the indexer to the default state: sets the current mode to IDLE and restarts the timer. Should be called when the robot is enabled to ensure that the indexer starts in a known state.
  public void init() {
    currMode = Mode.IDLE;
    isSpoolingUp = false;
    indexTimer.restart();
  }

  // Runs code every 20ms: if the current state is INDEX, then run the indexer motors at the specified voltage for 2.5 seconds, then reverse the motors for 0.5 seconds to help unjam any stuck fuel. If the current state is IDLE, then stop the motors and reset the timer.
  public void periodic() {
    switch (currMode) {
      case INDEX: // Runs the indexer motors at the specified voltage for 2.5 seconds, then reverses the motors for 0.5 seconds to help unjam any stuck fuel.
        if (indexTimer.get() < 2.5) {
          hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(indexVoltage).withEnableFOC(true));
        } else if (indexTimer.get() < 3.0) {
          hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(-indexVoltage).withEnableFOC(true));
        } else {
          indexTimer.restart(); // Restarts the timer to repeat the cycle of running forward for 2.5 seconds and then reversing for 0.5 seconds. This will continue until the mode is changed to IDLE.
        }
        shooterIndexMotor.setControl(shooterIndexMotorVelocityRequest.withVelocity(indexRPM).withEnableFOC(true));
      break;

      case IDLE: // Stops the motors.
        indexTimer.restart();
          hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
        if (isSpoolingUp) {
          shooterIndexMotor.setControl(shooterIndexMotorVelocityRequest.withVelocity(indexRPM).withEnableFOC(true));
        } else {
          shooterIndexMotor.setControl(shooterIndexMotorVelocityRequest.withVelocity(0.0).withEnableFOC(true));
        }
      break;
    }
  }

  // Sets the current state to INDEX, which will cause the indexer to run in the periodic method.
  public void start() {
    currMode = Mode.INDEX;
  }

  // Sets the current state to IDLE, which will cause the indexer to stop in the periodic method.
  public void stop() {
    currMode = Mode.IDLE;
  }

  public void spoolUp() {
    isSpoolingUp = true;
  }

  public void spoolDown() {
    isSpoolingUp = false;
  }

  // Sets the voltage that the indexer motors will run at when the indexer is running.
  public void setIndexRPM(double RPM) {
    indexRPM = RPM;
    indexVoltage = RPM/4000.0 * 12.0;
  }

  // Returns the current mode that the indexer is in.
  public Mode getMode() {
    return currMode;
  }

  // Updates the SmartDashboard with the current mode and voltage of the indexer. Useful for debugging and tuning.
  public void updateDash() {
    //SmartDashboard.putString("Indexer getMode", getMode().toString());
    //SmartDashboard.putNumber("Indexer voltage", indexVoltage);
  }

  public void simulationPeriodic() {
    
  }

  // Configures the indexer motors with the appropriate settings for our robot. Sets the neutral mode to brake, sets the motor direction based on the invert parameter, and configures current limits for the motor.
  private void configIndexMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise.

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction in the swerve wheel.

    // Configures the current limits for the motor. Supply current limit is the maximum current that can be drawn from the battery. Stator current limit is the maximum current that can be drawn by the motor windings. 
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120.0;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}