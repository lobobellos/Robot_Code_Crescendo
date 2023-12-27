// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PDP;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
  private final PDP pdp = new PDP(PDPConstants.deviceID);

  private final Command resetRotation = new ParallelCommandGroup(gyro.zero(),driveBase.resetRotation());

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //register commands
    NamedCommands.registerCommand("resetRotation", resetRotation);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveBase.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                driveBase.drive(
                    -m_driverController.getLeftY(),
                    -m_driverController.getLeftX(),
                    -m_driverController.getRightX(),
                    true),
            driveBase));

    pdp.setDefaultCommand(
        pdp.updatePDP()
    );


    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureButtonBindings() {

    SmartDashboard.putData("Potato", new PathPlannerAuto("lilTurn"));


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
