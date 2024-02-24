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

public class AlignToAmp extends RunCommand {

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
      AutoAlignmentConstants.MovementPID.kP,
      AutoAlignmentConstants.MovementPID.kI,
      AutoAlignmentConstants.MovementPID.kD);

  private static final SimpleMotorFeedforward rotationFF = new SimpleMotorFeedforward(
      AutoAlignmentConstants.MovementFF.kS,
      AutoAlignmentConstants.MovementFF.kV,
      AutoAlignmentConstants.MovementFF.kA);

  public AlignToAmp(Limelight limelight, DriveSubsystem drive) {
    super(
        () -> {
          Pose2d pose = limelight.getTargetPose();
          double rot = rotationPid.calculate(pose.getRotation().getDegrees())
              + rotationFF.calculate(pose.getRotation().getDegrees());
          SmartDashboard.putNumber("rot FF output", rot);
          drive.drive(0, 0, rot, false);
        });

    positionXPid.setSetpoint(AutoAlignmentConstants.positionSetpointX);
    positionXPid.setTolerance(AutoAlignmentConstants.positionTolerance);

    positionYPid.setSetpoint(AutoAlignmentConstants.positionSetpointY);
    positionYPid.setTolerance(AutoAlignmentConstants.positionTolerance);

    rotationPid.setSetpoint(AutoAlignmentConstants.rotationSetpoint.getDegrees());
    rotationPid.setTolerance(AutoAlignmentConstants.rotationTolerance.getDegrees());

    this.limelight = limelight;

    addRequirements(limelight, drive);

    System.out.println("start");
  }

  public boolean isFinished() {
    System.out.println("alignment finished");
    return rotationPid.atSetpoint() || !limelight.validTargetExists();
  }

}
