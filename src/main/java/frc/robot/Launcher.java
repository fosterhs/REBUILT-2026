package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {
  private static final double PSI_Scale = 49.8;
  private static final double PSI_Offset = -26.1;
  private final AnalogPotentiometer voltRead = new AnalogPotentiometer(0, 5, 0); //potentiometer on analog port 0
  private double PSISetPoint = 30.0; 

  private final VictorSPX motor1 = new VictorSPX(5); //Activates VictorSPX on CAN ID 5
  private final VictorSPX motor2 = new VictorSPX(2); //Activates VictorSPX on CAN ID 2
  private final CANdle launcherCandle = new CANdle(0, "canivore"); // Initializes the lights on the front of the robot.

  private final RGBWColor offColor = new RGBWColor(0, 0, 0); // Represents LEDs that are off.
  private final RGBWColor redColor = new RGBWColor(255, 0, 0); // Represents red LEDs.
  private final RGBWColor greenColor = new RGBWColor(0, 255, 0); // Represents green LEDs.
  private final RGBWColor blueColor = new RGBWColor(0, 0, 255); // Represents blue LEDs.

  //Constructor for the Launcher class
  public Launcher() {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
  }

  public double getPSI() {
    return (PSI_Scale * voltRead.get()) + PSI_Offset; //Calculates PSI from potentiometer voltage
  }

  public double getVolt() {
    return voltRead.get();
  }

  public void setPSI(double desiredPSI) {
    PSISetPoint = desiredPSI;
  }

  private void setShootingSolenid(Boolean ShootingSolenid) {
    if (ShootingSolenid) {
      motor1.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
  }

  private void setFillingSolenoid(Boolean fillSolenoid) {
    if (fillSolenoid) {
      motor1.set(VictorSPXControlMode.PercentOutput, 1.0);
    } else {
      motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
  }

  public void periodic() {

    if (getPSI() < PSISetPoint) {
      setFillingSolenoid(true);
    } else if (PSISetPoint <= getPSI()) {
      setFillingSolenoid(false);
    }

  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Launcher PSI", getPSI());
    SmartDashboard.putNumber("Volt Read", getVolt());
  }
}