package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Limelight;

public class AlignToAmp extends SequentialCommandGroup {

  Limelight limelight;

  private static final PIDController positionXPid = new PIDController(
      AutoAlignmentConstants.MovementPID.kP,
      AutoAlignmentConstants.MovementPID.kI,
      AutoAlignmentConstants.MovementPID.kD);
  private static final PIDController positionYPid = new PIDController(
      AutoAlignmentConstants.MovementPID.kP,
      AutoAlignmentConstants.MovementPID.kI,
      AutoAlignmentConstants.MovementPID.kD);

  private static final SimpleMotorFeedforward positionXFF = new SimpleMotorFeedforward(
      AutoAlignmentConstants.MovementFF.kS,
      AutoAlignmentConstants.MovementFF.kV,
      AutoAlignmentConstants.MovementFF.kA);

  private static final SimpleMotorFeedforward positionYFF = new SimpleMotorFeedforward(
      AutoAlignmentConstants.MovementFF.kS,
      AutoAlignmentConstants.MovementFF.kV,
      AutoAlignmentConstants.MovementFF.kA);

  private static final PIDController rotationPid = new PIDController(
      AutoAlignmentConstants.RotationPID.kP,
      AutoAlignmentConstants.RotationPID.kI,
      AutoAlignmentConstants.RotationPID.kD);

  private static final SimpleMotorFeedforward rotationFF = new SimpleMotorFeedforward(
      AutoAlignmentConstants.RotationFF.kS,
      AutoAlignmentConstants.RotationFF.kV,
      AutoAlignmentConstants.RotationFF.kA);

  public AlignToAmp(Limelight limelight, DriveSubsystem drive) {
    super(
        new RunCommand(
            () -> {
              Pose2d pose = limelight.getTargetPose();
              double rot = (-rotationPid.calculate(pose.getRotation().getDegrees()));
              SmartDashboard.putNumber("rot  output", rot);
              drive.drive(0, 0, rot , false);
            },
            drive) {
          private int setpointCounter = 0;

          public boolean isFinished() {
            if (rotationPid.atSetpoint())
              setpointCounter++;
            else
              setpointCounter = 0;

            return setpointCounter >= 2 || !limelight.validTargetExists();
          }

          public void end(boolean interupted) {
            System.out.println("alignment finished");
          }
        },
        drive.resetRotation());

    positionXPid.setSetpoint(AutoAlignmentConstants.positionSetpointX);
    positionXPid.setTolerance(AutoAlignmentConstants.positionTolerance);

    positionYPid.setSetpoint(AutoAlignmentConstants.positionSetpointY);
    positionYPid.setTolerance(AutoAlignmentConstants.positionTolerance);

    rotationPid.setSetpoint(AutoAlignmentConstants.rotationSetpoint.getDegrees());
    rotationPid.setTolerance(AutoAlignmentConstants.rotationTolerance.getDegrees());
    rotationPid.enableContinuousInput(-180, 180);

    this.limelight = limelight;

    addRequirements(limelight, drive);

    System.out.println("start");
  }

}
