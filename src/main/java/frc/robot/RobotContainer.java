// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private final Pneumatics pneumatics = new Pneumatics();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();

  private final Command resetRotation = new ParallelCommandGroup(gyro.zero(), driveBase.resetRotation());
  private final Command togglePusher = new InstantCommand(pneumatics::toggleShooterPiston);
  private final Command toggleCompressor = new InstantCommand(pneumatics::toggleCompressor);
  private final Command toggleIntakeEnabled = new InstantCommand(intake::toggleEnabled);
  private final Command toggleElevatorEnabled = new InstantCommand(elevator::toggleEnabled);
  private final Command toggleIntakeAndElevator = new SequentialCommandGroup(toggleIntakeEnabled, toggleElevatorEnabled);
  private final Command raiseHook = hook.raiseHook();
  private final Command lowerHook = hook.lowerHook();
  private final Command runHookRaw = hook.runRaw();
  private final Command retractHookRaw = hook.retractRaw();

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

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveBase.setDefaultCommand(
        new RunCommand(() -> driveBase.drive(
            -m_driverController.getLeftY(),
            -m_driverController.getLeftX(),
            -m_driverController.getRightX(),
            true),
            driveBase));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    leds.start();
  }

  private void configureButtonBindings() {

    SmartDashboard.putData(new PathPlannerAuto("lilTurn"));

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

    // Config mechanism controller buttons
    m_MechanismController.a()
        .onTrue(togglePusher);
    m_MechanismController.b()
        .onTrue(toggleIntakeAndElevator);


    m_MechanismController.rightBumper()
    .onTrue(raiseHook);
    m_MechanismController.leftBumper()
    .onTrue(lowerHook);
    m_MechanismController.povDown()
    .onTrue(retractHookRaw);
    m_MechanismController.povUp()
    .onTrue(runHookRaw);

    // config multi-controller buttons

    m_driverController.start().or(m_MechanismController.start())
        .onTrue(toggleCompressor);

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
