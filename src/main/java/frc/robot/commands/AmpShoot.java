package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends ParallelCommandGroup{

    //parallel
        //riun flywheel
        //sequential
            // wait for flywheel to be up to speed
            // activate solenoid
            // wait?
            // toggle solenoid
    public AmpShoot(Shooter shooter,Pneumatics pneumatics ){
        
    }

}