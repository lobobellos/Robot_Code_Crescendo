package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;

public class ZeroAll extends ParallelCommandGroup {
    public ZeroAll(DriveSubsystem drive, Gyro gyro){
        super(gyro.zero(),drive.resetRotation());
    }
}
