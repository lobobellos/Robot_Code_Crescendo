package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

  public static final AHRS ahrs = new AHRS(SPI.Port.kMXP); 

  public Gyro() {
    //recalibrate();
    
    addChild("imu", ahrs);
  }

  public InstantCommand zero() {
    return new InstantCommand(ahrs::zeroYaw, this);
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
