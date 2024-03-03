package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Pneumatics;

public class SolenoidOneShot extends SequentialCommandGroup{
    
 public SolenoidOneShot(Pneumatics pneumatics){
    super(
        new InstantCommand(()->pneumatics.setShooterPiston(true)),
        new WaitCommand(0.25),
        new InstantCommand(()->pneumatics.setShooterPiston(false))
    );
 }
}
