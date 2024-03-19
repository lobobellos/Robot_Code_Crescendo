package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

  public static final AHRS ahrs = new AHRS(SPI.Port.kMXP); 

  public Gyro() {
    zero();
    
    addChild("imu", ahrs);
  }

  public void zero(){
    ahrs.zeroYaw();
  }

  public Command zeroCommand() {
    return Commands.runOnce(this::zero);
  }

  public void recalibrate() {
    ahrs.reset();
  }

  public Rotation2d getRotation() {
    return ahrs.getRotation2d();
  }

  // returns angular velocity in Rotation2d per second
  public Rotation2d getAngularVelocity() {
    return Rotation2d.fromDegrees(ahrs.getRawGyroY());
  }
}
