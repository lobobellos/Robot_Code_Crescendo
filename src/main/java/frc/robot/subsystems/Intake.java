package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotorTop = new CANSparkMax(
      IntakeConstants.topMotorID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax intakeMotorBottom = new CANSparkMax(
      IntakeConstants.BottomMotorID,
      CANSparkMax.MotorType.kBrushless);

  private double kSpeedPercent = 0;

  public Intake() {
    intakeMotorTop.setInverted(IntakeConstants.kTopInverted);
    intakeMotorBottom.setInverted(IntakeConstants.kBottomInverted);
    this.setDefaultCommand(new RunCommand(this::run, this));
  }

  public void toggleEnabled() {
    if(kSpeedPercent == 0) {
      kSpeedPercent = IntakeConstants.kSpeedPercent;
    } else {
      kSpeedPercent = 0;
    }
  }

  public void run() {
    intakeMotorTop.set(kSpeedPercent);
    intakeMotorBottom.set(kSpeedPercent);
  }

  public void stop() {
    intakeMotorTop.set(0);
    intakeMotorBottom.set(0);
  }

  public void setSpeed(double target){
    kSpeedPercent = MathUtil.clamp(target,0,1);
  }
}
