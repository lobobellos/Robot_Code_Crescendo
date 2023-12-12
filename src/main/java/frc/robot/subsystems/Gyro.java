package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

  public static final ADIS16470_IMU imu = new ADIS16470_IMU();

  public Gyro() {
    recalibrate();
  }

  public void zero() {
    imu.reset();
  }

  public void recalibrate() {
    imu.calibrate();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(imu.getAngle());
  }
}
