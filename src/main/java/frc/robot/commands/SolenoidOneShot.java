package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Pneumatics;

public class SolenoidOneShot extends SequentialCommandGroup{
    
 public SolenoidOneShot(Pneumatics pneumatics){
    super(
        pneumatics.setShooterPistonUpCommand(),
        new WaitCommand(0.25),
        pneumatics.setShooterPistonDownCommand()
    );
 }
}
