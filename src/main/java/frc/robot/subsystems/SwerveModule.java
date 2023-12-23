// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.DrivePID;
import frc.robot.Constants.ModuleConstants.TurningPID;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveController;

  public final double turningEncoderOffset;

  public double velocitySetpoint = 0;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      TurningPID.kP,
      TurningPID.kI,
      TurningPID.kD,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeed,
          ModuleConstants.kMaxModuleAngularAcceleration));

  public double FF = DrivePID.kFF;
  public double kP = DrivePID.kP;
  public double kD = DrivePID.kD;

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

    m_driveMotor.setSmartCurrentLimit(
        CurrentLimits.driveMotorStall,
        CurrentLimits.driveMotorFree);
    m_turningMotor.setSmartCurrentLimit(
        CurrentLimits.turningMotorStall,
        CurrentLimits.turningMotorFree);

    m_driveMotor.setInverted(driveEncoderReversed);
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor.setIdleMode(IdleMode.kCoast); // TODO: turn these back to brake

    m_driveController.setFF(DrivePID.kFF, 0);
    m_driveController.setP(DrivePID.kP, 0);
    m_driveController.setD(DrivePID.kD, 0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kdrivePositionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kdriveVelocityConversionFactor);

    // Set the position offset (in rotations) for the turning
    // encoder.
    this.turningEncoderOffset = turingEncoderOffset;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    addChild("steering pid", m_turningPIDController);
    addChild("turning encoder", m_turningEncoder);
  }

  public Rotation2d getTurningRotation() {
    return Rotation2d.fromRotations(m_turningEncoder.getAbsolutePosition() + turningEncoderOffset);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getTurningRotation());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getTurningRotation());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getTurningRotation());

    // Calculate the drive output from the drive PID controller.
    m_driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0);
    this.velocitySetpoint = state.speedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurningRotation().getRadians(),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public void periodic() {
    m_driveController.setFF(FF, 0);
    m_driveController.setP(kP, 0);
    m_driveController.setD(kD, 0);

    SmartDashboard.putNumber("encoder/offset " + m_turningMotor.getDeviceId(), getTurningRotation().getRotations());
  }
}
