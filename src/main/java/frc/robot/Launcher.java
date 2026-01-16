package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class Launcher {
  private static final double PSI_Scale = 49.8;
  private static final double PSI_Offset = -26.1;
  private final AnalogPotentiometer voltRead = new AnalogPotentiometer(0, 5, 0); //potentiometer on analog port 0

  private final Timer solenoidTimer = new Timer();

  private double PSISetPoint = 30.0; 
  private double PSIMargin = 1.0;
  private double PSIsafe = 5.0;
  private final VictorSPX motor1 = new VictorSPX(1); //Activates VictorSPX on CAN ID 5
  private final VictorSPX motor2 = new VictorSPX(0); //Activates VictorSPX on CAN ID 2


  public enum Mode {safe, launching, filling, idle}; // A list containing all the possible modes of the launcher.
  private Mode currState = Mode.idle; // Stores the current mode of the launcher.

  // Constructor for the Launcher class
  public Launcher() {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
  }

  // Called once in robotInit() on robot startup. Determines whether the launcher should start in safe mode or idle mode based on the tank pressure.
  // Start in safe mode if the tank pressure is less than PSIMargin. Otherwise start in idle mode.
  public void init() {
    if(getPSI() < PSIMargin){
      currState = Mode.safe;
    } else {
      currState = Mode.idle;
    }
  }

  // Operates the launcher. Expect the user to call this function each period in teleop(). This function does the following, in order of priority:
  // 1. If in the safe state, close both the fill and launch solenoids. The user should not be able to do anything until they exit the safe state into the idle state.
  // 2. If in the launching state, open the shooting solenoid for exactly 2 seconds. Return to the idle state afterwards.
  // 3. If in the filling state, open the fill solenoid until desiredPSI is reached, but no longer than 10 seconds. Return to the idle state afterwards.
  // 4. If in the idle state, close both the fill and launch solenoids.
  public void periodic() {
    switch(currState) {
      case safe:
        setFillingSolenoid(false);
        setLaunchingSolenid(false);
        if (getPSI() > PSIsafe) {
          currState = Mode.idle;
        }
      break;

      case launching:
        setLaunchingSolenid(true);
        setFillingSolenoid(false);
        if (solenoidTimer.get() > 10.0){
         setLaunchingSolenid(false);
         setFillingSolenoid(false);
         currState = Mode.idle;
        }
      break;

      case filling:
        setFillingSolenoid(true);
        setLaunchingSolenid(false);
        if (getPSI() > getDesiredPSI() || solenoidTimer.get() >= 10.0) {
          setFillingSolenoid(false);
          setLaunchingSolenid(false);
          currState = Mode.idle;
        }
      break;

      case idle:
        setFillingSolenoid(false);
        setLaunchingSolenid(false);
      break;
    }

  //if(currState == case1) {
  //  do case 1 stuff
  //} else if(currState == case 2) {
  //  do case 2 stuff
  //} else {
  //  do default stuff
  //}
  }

  // Allows the user to command the launcher to launch a t-shirt. Refuse to launch if safe mode is enabled.
  public void launch() {
    if (currState != Mode.safe) {
      solenoidTimer.restart();
      currState = Mode.launching;
    }
  }

  // Allows the user to command the launcher to fill the tank to the desiredPSI. Refuse to fill if safe mode is enabled, or if the launcher is currently launching a t-shirt.
  public void fill() {
    if (currState != Mode.launching && currState != Mode.safe){
      solenoidTimer.restart();
      currState = Mode.filling;
    }
  }

  // Allows the user to command the launcher into safe mode, where no launch or fill commands will be executed. The user can only trigger safe mode if the PSI is less than PSIMargin.
  public void safe(boolean safeMode) {
    if  (safeMode && getPSI() < PSIMargin) {
     currState = Mode.safe;
    } 

    if (!safeMode) {
      currState = Mode.idle;
    }
  }

  // Returns the current mode of the launcher.
  public Mode getMode() {
    return currState;
  }

  // 
  public void setPSI(double desiredPSI) {
    PSISetPoint = desiredPSI;
  }

  // Returns the value of the desiredPSI
  public double getDesiredPSI() {
    return PSISetPoint;
  }
  
  // Calculates PSI from potentiometer voltage
  public double getPSI() {
    return (PSI_Scale * voltRead.get()) + PSI_Offset; 
  }

  // Returns the Voltage value from voltRead.get
  public double getVolt() {
    return voltRead.get();
  }


  public void updateDash() {
    SmartDashboard.putNumber("Launcher PSI", getPSI());
    SmartDashboard.putNumber("Volt Read", getVolt());
    SmartDashboard.putNumber("Solenoid Timer", solenoidTimer.get());
    SmartDashboard.putNumber("Desired PSI", getDesiredPSI());
    switch(currState){
      case safe:
      SmartDashboard.putString("Launcher Mode", "safe");
      break;
      case launching:
      SmartDashboard.putString("Launcher Mode", "launching");
      break;
      case filling:
      SmartDashboard.putString("Launcher Mode", "filling");
      break;
      case idle:
      SmartDashboard.putString("Launcher Mode", "idle");
      break;
    }
  }

  // Sets the Launching Solenid to open(true) or close(false)
  private void setLaunchingSolenid(Boolean ShootingSolenid) {
    if (ShootingSolenid) {
      motor1.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
  }

  // Sets the Filling Solenid to open(true) or close(false) 
  private void setFillingSolenoid(Boolean fillSolenoid) {
    if (fillSolenoid) {
      motor2.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor2.set(VictorSPXControlMode.PercentOutput, 0.0);
    } 
  }
}