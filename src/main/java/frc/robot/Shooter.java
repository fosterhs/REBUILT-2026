package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
  private final CANBus canivore = new CANBus("canivore"); // Creates a new CAN bus called "canivore". This is the name of the CAN bus that the shooter motors and hood encoder are connected to. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX hoodMotor = new TalonFX(9, canivore); // Creates a new TalonFX motor controller for the hood motor. The motor is assigned an ID of 9 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX shootMotorRight = new TalonFX(12, canivore); // Creates a new TalonFX motor controller for the right shooter motor. The motor is assigned an ID of 12 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final TalonFX shootMotorLeft = new TalonFX(11, canivore); // Creates a new TalonFX motor controller for the left shooter motor. The motor is assigned an ID of 11 on the CAN bus. Make sure to set this correctly based on your robot's wiring.
  private final CANcoder hoodEncoder = new CANcoder(28, canivore); // Creates a new CANcoder for the hood encoder. The encoder is assigned an ID of 28 on the CAN bus. Make sure to set this correctly based on your robot's wiring.

  private final StatusSignal<AngularVelocity> shooterVelocityRight; // Creates a new StatusSignal for the velocity of the right shooter motor. This will allow us to read the velocity of the motor in real-time and use it to determine if the motor is at the desired speed.
  private final StatusSignal<AngularVelocity> shooterVelocityLeft; // Creates a new StatusSignal for the velocity of the left shooter motor. This will allow us to read the velocity of the motor in real-time and use it to determine if the motor is at the desired speed.
  private final StatusSignal<Angle> hoodPosition; // Creates a new StatusSignal for the position of the hood motor. This will allow us to read the position of the motor in real-time and use it to determine if the motor is at the desired position.
  private final StatusSignal<Voltage> shooterVoltageRight; // Creates a new StatusSignal for the voltage of the right shooter motor. This will allow us to read the voltage of the motor in real-time and use it to monitor the health of the motor and ensure that it is not being overworked.

  private final MotionMagicVelocityVoltage shooterMotorRightVelocityRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true); // Creates a new VelocityVoltage control mode for the right shooter motor. This will allow us to set the velocity that we want to apply to the motor when it is running. The withEnableFOC(true) part enables field-oriented control, which can help improve the performance of the motor.
  private final Follower shooterMotorLeftFollowerRequest = new Follower(shootMotorRight.getDeviceID(), MotorAlignmentValue.Opposed); // Creates a new Follower control mode for the left shooter motor. This will allow us to set the left shooter motor to follow the right shooter motor, so that we only have to set the velocity for the right shooter motor and the left shooter motor will automatically match it. The MotorAlignmentValue.Opposed part means that the left shooter motor will run in the opposite direction of the right shooter motor, which is necessary for our shooter configuration.
  private final MotionMagicTorqueCurrentFOC hoodMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Creates a new MotionMagicTorqueCurrentFOC control mode for the hood motor. This will allow us to set the position that we want to apply to the motor when it is moving the hood. We will configure the PID values for this control mode in the configHoodMotor method.

  private final double rpmTol = 300.0; // Tolerance for checking if the shooter motors are at the desired RPM. This can be adjusted based on the performance of the motors and the requirements of the shooter.
  private final double hoodTol = 0.010; // Tolerance for checking if the hood is at the desired position. This can be adjusted based on the performance of the hood mechanism and the requirements of the shooter.
  public final double hoodMinPosition = 0.020; // The minimum position of the hood in hood rotations.
  public final double hoodMaxPosition = 0.115; // The maximum position of the hood in hood rotations.
  public final double maxFlywheelRPM = 5800.0; // The maximum RPM of the flywheel.
  private double shootingRPM = 3000.0; // Initializes the desired shooting RPM of the shooter motors. 
  private double desiredHoodPosition = hoodMinPosition; // Initializes the desired position of the hood. 
  private boolean isSpunUp = false;

  // Simulation
  private final TalonFXSimState hoodMotorSim = hoodMotor.getSimState();
  private final TalonFXSimState shootMotorRightSim = shootMotorRight.getSimState();
  private final TalonFXSimState shootMotorLeftSim = shootMotorLeft.getSimState();
  private final CANcoderSimState hoodEncoderSim = hoodEncoder.getSimState();
  private double desiredRPMSim = 0;
  public static final double hoodGearRatio = 1;

  // Constructor for the Shooter class. This is where we will configure the shooter motors and hood encoder with the appropriate settings for our robot. We will set the neutral mode to brake, set the motor direction based on the invert parameter, configure current limits for the motors, and configure the PID values for the hood motor. We will also optimize bus utilization for the motors and encoder to improve performance.
  public Shooter() {
    configHoodEncoder(hoodEncoder, 0.378662109375, true);
    configShootMotor(shootMotorRight, true);
    configShootMotor(shootMotorLeft, false); 
    configHoodMotor(hoodMotor, hoodEncoder, false);
    shooterVelocityRight = shootMotorRight.getVelocity();
    shooterVelocityLeft = shootMotorLeft.getVelocity();
    hoodPosition = hoodEncoder.getAbsolutePosition();
    shooterVoltageRight = shootMotorRight.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocityRight, shooterVelocityLeft, hoodPosition, shooterVoltageRight);
    ParentDevice.optimizeBusUtilizationForAll(shootMotorLeft, hoodMotor, hoodEncoder);
  }

  // Resets the shooter to the default state: sets the current mode to IDLE, stops the shooter motors, and lowers the hood. Should be called when the robot is enabled to ensure that the shooter starts in a known state.
  public void init() {
    spinDown();
    lowerHood();
  }
  
  // Runs code every 20ms: if the current state is SHOOT, then run the shooter motors at the specified shooting RPM. If the current state is IDLE, then stop the shooter motors. In both cases, set the left shooter motor to follow the right shooter motor, and set the hood motor to move to the desired hood position using MotionMagicTorqueCurrentFOC control mode.
  public void periodic() {
    if (isSpunUp) {
      shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(shootingRPM/60.0).withEnableFOC(true));
    } else {
      shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(0.0).withEnableFOC(true));
    }
    shootMotorLeft.setControl(shooterMotorLeftFollowerRequest); // Sets the left shooter motor to follow the right shooter motor.
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(desiredHoodPosition)); // Sets the hood motor to move to the desired hood position using MotionMagicTorqueCurrentFOC control mode.
  }

  // Sets the current state to SHOOT, which will cause the shooter to run in the periodic method at the specified shooting RPM.
  public void spinUp() {
    isSpunUp = true;
    desiredRPMSim = shootingRPM;
  }

  // Sets the current state to IDLE, which will cause the shooter to stop in the periodic method.
  public void spinDown() {
    isSpunUp = false;
    desiredRPMSim = 0;
  }

  // Sets the desired shooting RPM of the shooter motors. If the specified RPM is above the maximum RPM, it will be set to the maximum RPM. If the specified RPM is below the minimum RPM, it will be set to the minimum RPM. Otherwise, it will be set to the specified RPM.
  public void setShootingRPM(double rpm) {
    if (rpm > maxFlywheelRPM) {
      shootingRPM = maxFlywheelRPM;
    } else if (rpm < 0.0) {
      shootingRPM = 0.0;
    } else {
      shootingRPM = rpm;
    }
  }

  // Sets the desired position of the hood. If the specified position is above the maximum position, it will be set to the maximum position. If the specified position is below the minimum position, it will be set to the minimum position. Otherwise, it will be set to the specified position.
  public void setHoodPosition(double position) {
    if (position < hoodMinPosition) {
      desiredHoodPosition = hoodMinPosition;
    } else if (position > hoodMaxPosition) {
      desiredHoodPosition = hoodMaxPosition;
    } else {
      desiredHoodPosition = position;
    }
  }

  // Sets the desired position of the hood to the minimum position, which will lower the hood.
  public void lowerHood() {
    desiredHoodPosition = hoodMinPosition;
  }

  // Returns true or false based on whether the hood is near the desired position.
  public boolean hoodIsInPosition() {
    // TODO: Fix simulationPeriodic
    if (Robot.isSimulation()) return true;

    return Math.abs(desiredHoodPosition - getHoodPosition()) < hoodTol;
  }

  // Returns the current position of the hood in hood rotations.
  public double getHoodPosition() {
    return hoodPosition.refresh().getValueAsDouble();
  }

  // Returns true or false based on whether the shooter motors are near the desired shooting RPM.
  public boolean flywheelIsAtSpeed() {
    return Math.abs(shootingRPM - getRightFlywheelMotorRPM()) < rpmTol && Math.abs(shootingRPM - getLeftFlywheelMotorRPM()) < rpmTol;
  }

  // Returns the left shooter motor velocity in RPM (Rotations Per Minute)
  public double getLeftFlywheelMotorRPM() {
    return shooterVelocityLeft.refresh().getValueAsDouble()*60.0;
  }

  // Returns the right shooter motor velocity in RPM (Rotations Per Minute)
  public double getRightFlywheelMotorRPM() {
    return shooterVelocityRight.refresh().getValueAsDouble()*60.0;
  }

  // Returns true or false based on whether the shooter motors are near the desired shooting RPM and the hood is near the desired position. This can be used to determine if the shooter is ready to shoot fuel.
  public boolean isReady() {
    return flywheelIsAtSpeed() && hoodIsInPosition();
  }

  // Updates the SmartDashboard with the current mode, shooting RPM, hood position, and whether the shooter is at speed and in position. Useful for debugging and tuning.
  public void updateDash() {
    //SmartDashboard.putNumber("Shooter getRightShooterRPM", getRightFlywheelMotorRPM());
    //SmartDashboard.putNumber("Shooter getLeftShooterRPM", getLeftFlywheelMotorRPM());
    SmartDashboard.putBoolean("Shooter shooterIsAtSpeed", flywheelIsAtSpeed());
    //SmartDashboard.putNumber("Shooter shootingRPM", shootingRPM);
    //SmartDashboard.putNumber("Shooter getHoodPosition", getHoodPosition());
    SmartDashboard.putBoolean("Shooter hoodIsInPosition", hoodIsInPosition());
    //SmartDashboard.putNumber("Shooter desiredHoodPosition", desiredHoodPosition);
    SmartDashboard.putBoolean("Shooter isReady", isReady());
    //SmartDashboard.putBoolean("Shooter flywheelIsReady", flywheelIsReady());
    //SmartDashboard.putBoolean("Shooter flywheelIsAtSpeed", flywheelIsAtSpeed());
    //SmartDashboard.putString("Shooter getMode", getMode().toString());
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

  // Configures the shooter motors with the appropriate settings for our robot. Sets the neutral mode to brake, sets the motor direction based on the invert parameter, configures current limits for the motor, and configures the PID values for velocity control. These settings are important to ensure that the shooter motors perform well and are protected from damage.
  private void configShootMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied. This is important for the shooter motors to help them maintain their speed and position when they are not being actively powered.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise. This should be set based on the physical configuration of the motor on our robot to ensure that positive voltage makes the shooter spin in the correct direction to shoot fuel.

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.20; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 6.0; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.012; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/60.0; // Units: roations per second.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 2.0*5800.0/60.0; // Units: rotations per second per second. 

    // Current limit configuration. These settings will help protect the motors from drawing too much current and potentially damaging themselves or the electrical system of the robot. 
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 70.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Applies the configuration to the motor with a timeout of 0.03 seconds. This will set all the settings that we specified in the motorConfigs object to the motor.
  }

  // Configures the hood motor with the appropriate settings for our robot. Sets the neutral mode to brake, sets the motor direction based on the invert parameter, configures current limits for the motor, and configures the PID values for MotionMagicTorqueFOC closed-loop control. These settings are important to ensure that the hood motor performs well and is protected from damage.
  private void configHoodMotor(TalonFX motor, CANcoder encoder, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied. This is important for the hood motor to help it maintain its position when it is not being actively powered.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise. This should be set based on the physical configuration of the motor on our robot to ensure that positive voltage makes the hood move in the correct direction to raise and lower.

    motorConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID(); // Sets the ID of the remote sensor that will be used for feedback in closed-loop control. In this case, we are using the hood encoder as the feedback device for the hood motor, so we set this to the device ID of the hood encoder.
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // Sets the source of the feedback sensor for closed-loop control. Since we are using the hood encoder as the feedback device, we set this to FusedCANcoder, which means that the motor will use both the integrated sensor and the remote CANcoder sensor for feedback.
    motorConfigs.Feedback.RotorToSensorRatio = 211.68; // The ratio of motor rotations to sensor rotations. This should be set based on the gearing between the motor and the hood mechanism.

    // Current limit configuration. These settings will help protect the motor from drawing too much current and potentially damaging itself or the electrical system of the robot.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 10.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 30.0;

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*211.68); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*211.68); // Units: roations per second.

    motor.getConfigurator().apply(motorConfigs, 0.03); // Applies the configuration to the motor with a timeout of 0.03 seconds. This will set all the settings that we specified in the motorConfigs object to the motor.
  }

  // Configures the hood encoder with the appropriate settings for our robot. Sets the absolute sensor discontinuity point, magnet offset, and sensor direction based on the physical configuration of the encoder on our robot. These settings are important to ensure that the encoder readings are accurate and consistent with the actual position of the hood.
  private void configHoodEncoder(CANcoder encoder, double offset, boolean invert) {
    CANcoderConfiguration sensorConfigs = new CANcoderConfiguration(); // Creates a new configuration object for the CANcoder. This object will hold all the settings that we want to apply to the encoder.

    sensorConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // The point at which the sensor's absolute position reading discontinuously jumps from 1.0 back to 0.0. This should be set based on the physical configuration of the encoder on our robot. 
    sensorConfigs.MagnetSensor.MagnetOffset = offset; // The offset to apply to the sensor's absolute position reading to align it with the actual position of the hood. This should be set based on the physical configuration of the encoder on our robot.
    sensorConfigs.MagnetSensor.SensorDirection = invert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive; // The direction that the sensor counts as positive. This should be set based on the physical configuration of the encoder on our robot.

    encoder.getConfigurator().apply(sensorConfigs, 0.03); // Applies the configuration to the encoder with a timeout of 0.03 seconds. This will set all the settings that we specified in the sensorConfigs object to the encoder.
  }
}