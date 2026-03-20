package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

class SwerveModule {
  public static final double correctionFactor = 0.98; // Factor that corrects for real-world deviations from the odometry calculated position of the robot. These can be caused by things like tread wear. Set this value to 1, then make the robot follow a 1 meter path in auto. Set this value to the distance the robot actually traveled.
  public static final double wheelCirc = 4.0*0.0254*Math.PI; // Circumference of the wheel. Unit: meters
  public static final double turnGearRatio = 18.75; // Turn motor rotor rotations per turn rotation of the swerve wheel.
  public static final double driveGearRatio = 75.0/14.0; // Drive motor rotor rotations per drive rotation of the swerve wheel.
  public static final double maxVel = 5800.0*wheelCirc*correctionFactor/(60.0*driveGearRatio); // The maximum speed of the swerve module in meters per second.
  public final CANcoder wheelEncoder; // The CANcoder that measures the angle of the swerve wheel.
  public final TalonFX driveMotor; // The Kraken X60 motor that controls the driving of the swerve module.
  public final TalonFX turnMotor; // The Kraken X60 motor that controls the turning of the swerve module.
  private final CANBus bus; // The name of the CAN bus that the swerve module is connected to. Typically "rio" for the RoboRIO, or "canivore" for the CANivore.
  public final StatusSignal<Angle> driveMotorPosition; // Stores the position of the drive motor.
  public final StatusSignal<AngularVelocity> driveMotorVelocity; // Stores the velocity of the drive motor.
  public final StatusSignal<Angle> wheelEncoderPosition; // Stores the position of the wheel encoder.
  public final StatusSignal<AngularVelocity> wheelEncoderVelocity; // Stores the velocity of the wheel encoder.
  private final VelocityVoltage driveMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true); // Communicates velocity voltage velocity requests to the drive motor.
  private final MotionMagicTorqueCurrentFOC turnMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Communicates motion magic torque current FOC position requests to the turn motor.
  private SwerveModulePosition SMP = new SwerveModulePosition(); // Stores the current wheel position and drive motor position of the swerve module.
  private SwerveModuleState SMS = new SwerveModuleState(); // Stores the current velocity and angle of the swerve module.


  // Constructor for the SwerveModule class. Initializes the CANcoder, drive motor, and turn motor with the given IDs and configurations. Also sets up the status signals for the drive motor and wheel encoder, and optimizes bus utilization for the motors and encoder. The wheelEncoderZero parameter is used to set the zero position of the wheel encoder, which corresponds to the angle at which the swerve wheel is facing straight forward.
  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive, double wheelEncoderZero, String canbus) {
    bus = new CANBus(canbus);
    wheelEncoder = new CANcoder(encoderID, bus);
    configEncoder(wheelEncoder, wheelEncoderZero);
    turnMotor = new TalonFX(turnID, bus);
    configTurnMotor(turnMotor, true);
    driveMotor = new TalonFX(driveID, bus);
    configDriveMotor(driveMotor, invertDrive);
    driveMotor.setPosition(0.0, 0.03);
    driveMotorPosition = driveMotor.getPosition();
    driveMotorVelocity = driveMotor.getVelocity();
    wheelEncoderPosition = wheelEncoder.getAbsolutePosition();
    wheelEncoderVelocity = wheelEncoder.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, driveMotorPosition, driveMotorVelocity, wheelEncoderPosition, wheelEncoderVelocity);
    ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, wheelEncoder);

  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    SMP.angle = Rotation2d.fromDegrees(getWheelAngle());
    desiredState.optimize(SMP.angle); // Minimizes the amount a wheel needs to rotate by inverting the direction of the drive motor in some situations. 
    desiredState.cosineScale(SMP.angle); // Cosine compensation. If a wheel is not at its angular setpoint, its velocity setpoint is reduced.
    setAngle(desiredState.angle.getDegrees());
    setVel(desiredState.speedMetersPerSecond);
  }

  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    SMP.angle = Rotation2d.fromDegrees(getWheelAngle());
    SMP.distanceMeters = getDriveMotorPos();
    return SMP;
  }

  // Returns the velocity and angle of the module.
  public SwerveModuleState getSMS() {
    SMS.angle = Rotation2d.fromDegrees(getWheelAngle());
    SMS.speedMetersPerSecond = getDriveMotorVel();
    return SMS;
  }

  // Returns the current velocity of the wheel. Unit: meters per second
  public double getDriveMotorVel() {
    return driveMotorVelocity.getValueAsDouble()*wheelCirc*correctionFactor/driveGearRatio;
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getDriveMotorPos() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(driveMotorPosition, driveMotorVelocity, 0.02)*wheelCirc*correctionFactor/driveGearRatio;
  }

  // Returns the raw value of the wheel encoder. Range: -180 to 180 degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). CCW positive coordinate system.
  public double getWheelAngle() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(wheelEncoderPosition, wheelEncoderVelocity, 0.02)*360.0;
  }

  // Sets the velocity of the module. Units: meters per second
  private void setVel(double vel) {
    driveMotor.setControl(driveMotorVelocityRequest.withVelocity(vel*driveGearRatio/(wheelCirc*correctionFactor)));
  }

  // Sets the angle of the module. Units: degrees Can accept values outside of -180 to 180, corresponding to multiple rotations of the swerve wheel.
  private void setAngle(double angle) {
    turnMotor.setControl(turnMotorPositionRequest.withPosition(angle/360.0));
  }


  // Configures the swerve module's drive motor.
  private void configDriveMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise.

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction in the swerve wheel.

    // Current limit configuration. 
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 70.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Applies the configuration to the motor. 
  }

  // Configures the swerve module's turn motor.
  private void configTurnMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration(); // Creates a new configuration object for the motor. This object will hold all the settings that we want to apply to the motor.

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Sets the motor to brake mode, which means it will resist being moved when no power is applied.
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Sets the motor direction. If invert is true, then positive voltage will make the motor spin clockwise. If false, then positive voltage will make the motor spin counterclockwise.

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 5000.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*turnGearRatio); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*turnGearRatio); // Units: roations per second.

    // CANcoder feedback configurations.
    motorConfigs.Feedback.FeedbackRemoteSensorID = wheelEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = turnGearRatio;
    motorConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // Current limit configuration.
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0;

    motor.getConfigurator().apply(motorConfigs, 0.03); // Applies the configuration to the motor.
  }

  // Configures the swerve module's wheel encoder.
  private void configEncoder(CANcoder encoder, double wheelEncoderZero) {
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration(); // Creates a new configuration object for the encoder. This object will hold all the settings that we want to apply to the encoder.

    encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Sets the point at which the absolute position sensor value wraps around from 360 back to 0 degrees. 
    encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // Sets the direction of the sensor. If CounterClockwise_Positive, then the sensor value will increase as the wheel turns counterclockwise. If Clockwise_Positive, then the sensor value will increase as the wheel turns clockwise.
    encoderConfigs.MagnetSensor.MagnetOffset = wheelEncoderZero; // Sets the zero position of the encoder. This should be set to the angle at which the swerve wheel is facing straight forward. Adjust this value as needed based on your specific robot's swerve module design and how it is mounted on the robot.

    encoder.getConfigurator().apply(encoderConfigs, 0.03); // Applies the configuration to the encoder.
  }
}