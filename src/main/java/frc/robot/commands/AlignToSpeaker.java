package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignToSpeaker extends SequentialCommandGroup {

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

  public AlignToSpeaker(Limelight limelight, DriveSubsystem drive) {
    super(
        new RunCommand(
            () -> {
              Pose2d pose = limelight.getTargetPose();

              double rot = // rotationPid.atSetpoint() ? 0 :
                  (-rotationPid.calculate(pose.getRotation().getDegrees()));
              // double xVel = // positionXPid.atSetpoint() ? 0 :
              // (positionXPid.calculate(pose.getX())) - positionXFF.calculate(pose.getX());
              // double yVel = // positionYPid.atSetpoint() ? 0 :
              // (-positionYPid.calculate(pose.getY())) + positionYFF.calculate(pose.getY());

              SmartDashboard.putNumber("rot  output", rot);
              drive.alignmentDrive(0, 0, rot, pose.getRotation().times(-1));
            },
            drive) {

          public boolean isFinished() {
            // Pose2d pose = limelight.getTargetPose();

            // rotationPid.calculate(pose.getRotation().getDegrees());

            final boolean atSetpoint = (rotationPid.atSetpoint() &&
                positionXPid.atSetpoint() &&
                positionYPid.atSetpoint()) || !limelight.validTargetExists();
            SmartDashboard.putBoolean("atSetpoint", atSetpoint);
            return atSetpoint;
          }

          public void end(boolean interupted) {
            System.out.println("alignment finished");
          }

          public void initialize() {
            var p = limelight.getTargetPose();

            Pose2d setpoint;
            if (Math.abs(p.getRotation().getDegrees()) <= 30) {
              setpoint = AutoAlignmentConstants.SpeakerSetpoint.center;

            } else if (p.getRotation().getDegrees() < 0) {
              setpoint = AutoAlignmentConstants.SpeakerSetpoint.right;
            } else if (p.getRotation().getDegrees() > 0) {
              setpoint = AutoAlignmentConstants.SpeakerSetpoint.left;
            } else {
              this.cancel();
              return;
            }
            positionXPid.setSetpoint(setpoint.getX());
            positionYPid.setSetpoint(setpoint.getY());
            rotationPid.setSetpoint(setpoint.getRotation().getDegrees());
          }
        },
        drive.resetRotation());

    positionYPid.setTolerance(AutoAlignmentConstants.SpeakerSetpoint.positionTolerance);
    positionXPid.setTolerance(AutoAlignmentConstants.SpeakerSetpoint.positionTolerance);
    rotationPid.setTolerance(AutoAlignmentConstants.SpeakerSetpoint.rotationTolerance.getDegrees());
    rotationPid.enableContinuousInput(-180, 180);

    System.out.println("start");
  }

}
