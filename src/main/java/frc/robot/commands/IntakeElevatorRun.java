package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPivotConstants.jointPosition;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class IntakeElevatorRun extends SequentialCommandGroup {
	public IntakeElevatorRun(Intake intake, Elevator elevator, ArmPivot pivot) {
		super(
				pivot.setPositionCommand(jointPosition.starting),
				Commands.parallel(
						intake.runCommand(),
						elevator.runCommand()));

	}
}
