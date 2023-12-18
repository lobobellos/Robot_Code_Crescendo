// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.TurningPID;

public class SwerveModule extends SubsystemBase{
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveController; 

  private final boolean driveMotorReversed;


  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      TurningPID.kP,
      TurningPID.kI,
      TurningPID.kD,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderChannels   The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double turingEncoderOffset,
      boolean driveEncoderReversed) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMax.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMax.MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel);

    m_driveController = m_driveMotor.getPIDController();

    CANSparkMax.enableExternalUSBControl(false);
  
    this.driveMotorReversed = driveEncoderReversed;

    //m_driveController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);
    //m_driveController.setSmartMotionMaxAccel(1000,0);
    //m_driveController.setSmartMotionMaxVelocity(1000,0);
    //m_driveController.setSmartMotionAllowedClosedLoopError(10,0);

    m_driveController.setFF(0.0002,0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveConversionFactor);
    //m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kdriveConversionFactor);
    


    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    m_turningEncoder.setPositionOffset(turingEncoderOffset);
    // This is the the angle through an entire rotation (2 * pi)
    m_turningEncoder.setDistancePerRotation( 2* Math.PI);
    

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    addChild("steering pid", m_turningPIDController);
    addChild("turning encoder", m_turningEncoder);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocityConversionFactor(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    // Calculate the drive output from the drive PID controller.
    
    final double speedMetersPerSecond =  (driveMotorReversed? -1.0: 1.0)* state.speedMetersPerSecond;

    SmartDashboard.putNumber("m/s speed", speedMetersPerSecond);

    final double RPMspeed = speedMetersPerSecond *60 * (1.0/0.0508)* (1.0/6.75);

    SmartDashboard.putNumber("rpm speed", RPMspeed);

    double multiplier = 3.0;

    m_driveController.setReference(RPMspeed*multiplier,ControlType.kVelocity,0);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public void periodic(){
    SmartDashboard.putNumber("encoder "+m_driveMotor.getDeviceId() +"velocity",m_driveEncoder.getVelocity());
    SmartDashboard.putNumber("encoder "+m_driveMotor.getDeviceId() +"velocity setpoint",m_driveEncoder.getVelocity());

  }
}
