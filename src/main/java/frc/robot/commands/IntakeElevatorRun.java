package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class IntakeElevatorRun extends ParallelCommandGroup {
	public IntakeElevatorRun(Intake intake, Elevator elevator) {
		super(
				intake.runCommand(),
				elevator.runCommand());
	}
}
