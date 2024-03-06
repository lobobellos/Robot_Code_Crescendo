package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

public class SpeakerShoot extends ParallelCommandGroup {

	// parallel
	// run flywheel
	// sequential
	// wait for flywheel to be up to speed
	// shoot note
	public SpeakerShoot(Shooter shooter, Pneumatics pneumatics, DriveSubsystem drive) {
		super(
				Commands.race(
						shooter.runSpeakerCommand(),
						drive.stopCommand(),
						Commands.sequence(
								//Commands.waitUntil(shooter::atSetpoint),
								Commands.waitSeconds(ShooterConstants.timeToShoot),
								new SolenoidOneShot(pneumatics),
								Commands.waitSeconds(0.5))
						));
	}
}