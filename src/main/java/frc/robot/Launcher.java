package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {
  private static final double PSI_Scale = 49.8;
  private static final double PSI_Offset = -26.1;
  private final AnalogPotentiometer voltRead = new AnalogPotentiometer(0, 5, 0); //potentiometer on analog port 0
  private double PSISetPoint = 50.0; 
  private double PSIMargin = 1.0;

  private final VictorSPX motor1 = new VictorSPX(1); //Activates VictorSPX on CAN ID 5
  private final VictorSPX motor2 = new VictorSPX(0); //Activates VictorSPX on CAN ID 2

  private enum Mode {safe, launching, filling, idle}; // A list containing all the possible modes of the launcher.
  private Mode currState = Mode.idle; // Stores the current mode of the launcher.

  // Constructor for the Launcher class
  public Launcher() {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
  }

  // Called once in robotInit() on robot startup. Determines whether the launcher should start in safe mode or idle mode based on the tank pressure.
  // Start in safe mode if the tank pressure is less than PSIMargin. Otherwise start in idle mode.
  public void init() {

  }

  // Operates the launcher. Expect the user to call this function each period in teleop(). This function does the following, in order of priority:
  // 1. If in the safe state, close both the fill and launch solenoids. The user should not be able to do anything until they exit the safe state into the idle state.
  // 2. If in the launching state, open the shooting solenoid for exactly 2 seconds. Return to the idle state afterwards.
  // 3. If in the filling state, open the fill solenoid until desiredPSI is reached, but no longer than 10 seconds. Return to the idle state afterwards.
  // 4. If in the idle state, close both the fill and launch solenoids.
  public void periodic() {

  }

  // Allows the user to command the launcher to launch a t-shirt. Refuse to launch if safe mode is enabled.
  public void launch() {

  }

  // Allows the user to command the launcher to fill the tank to the desiredPSI. Refuse to fill if safe mode is enabled, or if the launcher is currently launching a t-shirt.
  public void fill() {

  }

  // Allows the user to command the launcher into safe mode, where no launch or fill commands will be executed. The user can only trigger safe mode if the PSI is less than PSIMargin.
  public void safe(boolean safeMode) {

  }

  // Returns the current mode of the launcher.
  public Mode getMode() {

  }

  // 
  public void setPSI(double desiredPSI) {
    PSISetPoint = desiredPSI;
  }

  // Returns the value of the desiredPSI
  public double getDesiredPSI() {
    
  }
  
  // Calculates PSI from potentiometer voltage
  public double getPSI() {
    return (PSI_Scale * voltRead.get()) + PSI_Offset; 
  }

  // 
  public double getVolt() {
    return voltRead.get();
  }


  public void updateDash() {
    SmartDashboard.putNumber("Launcher PSI", getPSI());
    SmartDashboard.putNumber("Volt Read", getVolt());
  }

  // 
  private void setShootingSolenid(Boolean ShootingSolenid) {
    if (ShootingSolenid) {
      motor1.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
  }

  // 
  private void setFillingSolenoid(Boolean fillSolenoid) {
    if (fillSolenoid) {
      motor2.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor2.set(VictorSPXControlMode.PercentOutput, 0.0);
    } 
  }
}