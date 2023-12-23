// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.RotationPID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftTurningEncoderOffset,
      DriveConstants.kFrontLeftDriveEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftTurningEncoderOffset,
      DriveConstants.kRearLeftDriveEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightTurningEncoderOffset,
      DriveConstants.kFrontRightDriveEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightTurningEncoderOffset,
      DriveConstants.kRearRightDriveEncoderReversed);

  private final SwerveModule[] modules = {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
  };

  // the field to send to shuffleboard
  private final Field2d field = new Field2d();

  // The gyro sensor
  private final Gyro gyro;

  private boolean isDemo = true;

  private final SwerveDriveOdometry m_odometry;

  private Rotation2d rotationSetpoint = new Rotation2d(0);

  private final PIDController rotationPID = new PIDController(
      RotationPID.kP,
      RotationPID.kI,
      RotationPID.kD);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Gyro gyro) {
    this.gyro = gyro;
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        gyro.getRotation(),
        modulePositions());

    addChild("frontLeft", m_frontLeft);
    addChild("frontRight", m_frontRight);
    addChild("rearLeft", m_rearLeft);
    addChild("rearRight", m_rearRight);
    addChild("field", field);

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    var driveTab = Shuffleboard.getTab("drive");
    driveTab.addDoubleArray("module velocities",
        () -> new double[] {
            m_frontLeft.getState().speedMetersPerSecond,
            m_frontLeft.velocitySetpoint
        });
  }

  public SwerveModulePosition[] modulePositions() {
    var arr = new SwerveModulePosition[4];
    for (var i = 0; i < modules.length; i++)
      arr[i] = modules[i].getPosition();
    return arr;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        gyro.getRotation(),
        modulePositions());
    var pose = getPose();
    field.setRobotPose(pose);
    SmartDashboard.putNumber("poseX", pose.getX());
    SmartDashboard.putNumber("poseY", pose.getY());
    SmartDashboard.putData("field", field);

    // Used for tuning. DO NOT DELETE!
    // var FF = SmartDashboard.getNumber("driveFF", DrivePID.kFF);
    // SmartDashboard.putNumber("driveFF", FF);
    // var kP = SmartDashboard.getNumber("driveP", DrivePID.kP);
    // SmartDashboard.putNumber("driveP", kP);
    // var kD = SmartDashboard.getNumber("driveD", DrivePID.kD);
    // SmartDashboard.putNumber("driveD", kD);
    // for (SwerveModule module : modules) {
    // module.FF = FF;
    // module.kP = kP;
    // module.kD = kD;
    // }

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        gyro.getRotation(),
        modulePositions(),
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    final var multiplier = isDemo ? DriveConstants.kDemoSpeedMetersPerSecond : DriveConstants.kMaxSpeedMetersPerSecond;
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.05) * multiplier;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.05) * multiplier;

    rot = MathUtil.applyDeadband(rot, 0.05) * OIConstants.rotationMultiplier;

    rotationSetpoint = rotationSetpoint.plus(new Rotation2d(rot));

    SmartDashboard.putNumber("rotation/add", rot);
    SmartDashboard.putNumber("rotation/setpoint", rotationSetpoint.getRadians());

    var rotationOutput = rotationPID.calculate(
        gyro.getRotation().getRadians(),
        rotationSetpoint.getRadians());

    SmartDashboard.putNumber("rotation/pidOutput", rotationOutput);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationOutput, gyro.getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationOutput));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public InstantCommand toggleDemoMode() {
    return new InstantCommand(() -> isDemo = !isDemo);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public InstantCommand resetRotation() {
    return new InstantCommand(() -> {
      rotationSetpoint = new Rotation2d();
    });
  }

  public InstantCommand turnTo(Rotation2d rot){
    return new InstantCommand(()->{rotationSetpoint = rot;});
  }

  public InstantCommand turnAmmount(Rotation2d rot){
    return new InstantCommand(()->{rotationSetpoint = rotationSetpoint.minus(rot);});
  }

  public void resetEncoders() {
    for (var module : modules) {
      module.resetEncoders();
    }
  }
}
