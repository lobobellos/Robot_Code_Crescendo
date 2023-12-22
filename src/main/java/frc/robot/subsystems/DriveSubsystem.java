// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
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

  //the field to send to shuffleboard
  private final Field2d field= new Field2d();

  // The gyro sensor
  private final Gyro gyro;

  private boolean isDemo = true;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry; 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Gyro gyro) {
    this.gyro = gyro;
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      gyro.getRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    addChild("frontLeft", m_frontLeft);
    addChild("frontRight", m_frontRight);
    addChild("rearLeft", m_rearLeft);
    addChild("rearRight", m_rearRight);

    addChild("field", field);
    

    var driveTab = Shuffleboard.getTab("drive");
    driveTab.addDoubleArray("module velocities",
    ()->new double[]{
      m_frontLeft.getState().speedMetersPerSecond,
      m_frontLeft.velocitySetpoint
    });
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        gyro.getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    var pose = getPose();
    field.setRobotPose(pose);
    SmartDashboard.putNumber("poseX",pose.getX());
    SmartDashboard.putNumber("poseY",pose.getY());

    SmartDashboard.putData("field", field);

    // 
    // var FF = SmartDashboard.getNumber("driveFF", DrivePID.kFF);
    // SmartDashboard.putNumber("driveFF", FF);
    // var kP = SmartDashboard.getNumber("driveP", DrivePID.kP);
    // SmartDashboard.putNumber("driveP", kP);
    // var kD = SmartDashboard.getNumber("driveD", DrivePID.kD);
    // SmartDashboard.putNumber("driveD", kD);
    // for (SwerveModule module : modules) {
    //   module.FF = FF;
    //   module.kP = kP;
    //   module.kD = kD;
    // }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        gyro.getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    final var multiplier = isDemo? DriveConstants.kDemoSpeedMetersPerSecond : DriveConstants.kMaxSpeedMetersPerSecond;
    xSpeed= MathUtil.applyDeadband(xSpeed, 0.05)*multiplier;
    ySpeed= MathUtil.applyDeadband(ySpeed, 0.05)*multiplier;
    rot= MathUtil.applyDeadband(rot, 0.05)*multiplier;
    
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));


    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    //putVelocities
    
  }

  public InstantCommand toggleDemoMode(){
    return new InstantCommand(()->isDemo = !isDemo);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }


}
