// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAmp;
import frc.robot.commands.IntakeElevatorRun;
import frc.robot.commands.SolenoidOneShot;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final Gyro gyro = new Gyro();
	private final DriveSubsystem driveBase = new DriveSubsystem(gyro);
	private final Hook hook = new Hook();
	private final Leds leds = new Leds();
	private final Limelight limelight = new Limelight("limelight");
	private final Pneumatics pneumatics = new Pneumatics();
	private final Intake intake = new Intake();
	private final Elevator elevator = new Elevator();
	private final Shooter shooter = new Shooter();

	// instant commands
	private final Command resetRotation =  driveBase.resetRotation();
	private final Command compressorEnable = pneumatics.enableCompressorCommand();
	private final Command compressorDisable = pneumatics.disableCompressorCommand();

	// run commands
	private final Command runHookRaw = new RunCommand(hook::runRaw, hook);
	private final Command retractHookRaw = new RunCommand(hook::retractRaw, hook);
	private final Command runShooter = new RunCommand(shooter::run, shooter);
	
	// command groups
	private final Command intakeElevatorRun = new IntakeElevatorRun(intake, elevator);
	private final Command solenoidOneShot = new SolenoidOneShot(pneumatics);
	private final AlignToAmp alignToAmp = new AlignToAmp(limelight, driveBase);


	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
	// the mechanism controller
	CommandXboxController m_MechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// register commands
		NamedCommands.registerCommand("resetRotation", resetRotation);
		NamedCommands.registerCommand("toggleIntakeAndElevator", intakeElevatorRun);
		NamedCommands.registerCommand("solenoidOneShot", solenoidOneShot);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		driveBase.setDefaultCommand(
				new RunCommand(() -> driveBase.joystickDrive(
						-m_driverController.getLeftY(),
						-m_driverController.getLeftX(),
						-m_driverController.getRightX(),
						true),
						driveBase));

		autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
		SmartDashboard.putData("Auto Mode", autoChooser);

		leds.start();

		CameraServer.startAutomaticCapture(0);
	}

	private void configureButtonBindings() {

		SmartDashboard.putData(new PathPlannerAuto("ampStart"));
		SmartDashboard.putData(new PathPlannerAuto("rollOut"));

		// Config driver controller buttons

		m_driverController.a()
				.onTrue(resetRotation);
		m_driverController.leftBumper()
				.and(m_driverController.rightBumper())
				.and(m_driverController.y())
				.onTrue(driveBase.toggleDemoMode());
		m_driverController.leftBumper()
				.onTrue(driveBase.turnAmmount(Rotation2d.fromRotations(-0.25)));
		m_driverController.rightBumper()
				.onTrue(driveBase.turnAmmount(Rotation2d.fromRotations(0.25)));

		m_driverController.b()
				.onTrue(alignToAmp);

		// Config mechanism controller buttons
		m_MechanismController.x()
				.onTrue(solenoidOneShot);
		m_MechanismController.b()
				.toggleOnTrue(intakeElevatorRun);
		m_MechanismController.a()
				.toggleOnTrue(runShooter);

		m_MechanismController.povDown()
				.whileTrue(retractHookRaw);
		m_MechanismController.povUp()
				.whileTrue(runHookRaw);

		// config multi-controller buttons

		m_driverController.rightTrigger(0.5)
				.onTrue(compressorEnable);

		m_driverController.leftTrigger(0.5)
				.onTrue(compressorDisable);

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser == null
				? new PrintCommand("uh... there's no auto, dude")
				: autoChooser.getSelected();
	}
}
