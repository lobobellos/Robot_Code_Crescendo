package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends ParallelCommandGroup {

	// parallel
	// run flywheel
	// sequential
	// wait for flywheel to be up to speed
	// shoot note
	public AmpShoot(Shooter shooter, Pneumatics pneumatics) {
		super(
				Commands.race(
						shooter.runAmpCommand(),
						Commands.sequence(
								Commands.waitUntil(shooter::atSetpoint),
								new SolenoidOneShot(pneumatics),
								Commands.waitSeconds(0.5))));
	}
}