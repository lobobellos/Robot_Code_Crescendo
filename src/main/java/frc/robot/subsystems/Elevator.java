package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax intakeMotorTop = new CANSparkMax(
      ElevatorConstants.topMotorID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax intakeMotorBottom = new CANSparkMax(
      ElevatorConstants.BottomMotorID,
      CANSparkMax.MotorType.kBrushless);

  private double kSpeedPercent = 0;

  public Elevator() {
    intakeMotorTop.setInverted(ElevatorConstants.kTopInverted);
    intakeMotorBottom.setInverted(ElevatorConstants.kBottomInverted);

    this.setDefaultCommand(new RunCommand(this::run, this));

  }

  public void toggleEnabled() {
    if(kSpeedPercent == 0) {
      kSpeedPercent = ElevatorConstants.kSpeedPercent;
    } else {
      kSpeedPercent = 0;
    }
  }

  public void run() {
    intakeMotorTop.set(kSpeedPercent);
    intakeMotorBottom.set(kSpeedPercent);
  }

  public void stop() {
    kSpeedPercent = 0;
  }

  public void setSpeed(double target){
    kSpeedPercent = MathUtil.clamp(target,0,1);
  }
}
