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

  //Constructor for the Launcher class
  public Launcher() {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
  }

  //Calculates PSI from potentiometer voltage
  public double getPSI() {
    return (PSI_Scale * voltRead.get()) + PSI_Offset; 
  }

  public double getVolt() {
    return voltRead.get();
  }

  public void setPSI(double desiredPSI) {
    PSISetPoint = desiredPSI;
  }

  public void setShootingSolenid(Boolean ShootingSolenid) {
    if (ShootingSolenid) {
      motor1.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
  }

  public void setFillingSolenoid(Boolean fillSolenoid) {
    if (fillSolenoid) {
      motor2.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor2.set(VictorSPXControlMode.PercentOutput, 0.0);
    } 
  }

  public void updateDash() {
    SmartDashboard.putNumber("Launcher PSI", getPSI());
    SmartDashboard.putNumber("Volt Read", getVolt());
  }
}