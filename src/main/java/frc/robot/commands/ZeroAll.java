package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll(DriveSubsystem drive, Gyro gyro){
        super(gyro.zeroCommand(),drive.resetRotation());
    }
}
