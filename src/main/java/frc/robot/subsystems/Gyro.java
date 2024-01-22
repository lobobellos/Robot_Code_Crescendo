package frc.robot.subsystems;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

  public static final ADIS16470_IMU imu = new ADIS16470_IMU();

  public Gyro() {
    //recalibrate();
    addChild("imu", imu);
  }

  public InstantCommand zero() {
    return new InstantCommand(()->imu.reset(), this);
  }

  public void recalibrate() {
    imu.calibrate();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kYaw));
  }

  // returns angular velocity in Rotation2d per second
  public Rotation2d getAngularVelocity() {
    return Rotation2d.fromDegrees(imu.getRate(IMUAxis.kYaw));
  }
}
