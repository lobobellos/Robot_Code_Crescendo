package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax MotorTop = new CANSparkMax(
      IntakeConstants.topMotorID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax MotorBottom = new CANSparkMax(
      IntakeConstants.BottomMotorID,
      CANSparkMax.MotorType.kBrushless);

  public Intake() {
    MotorTop.setInverted(IntakeConstants.kTopInverted);
    MotorBottom.setInverted(IntakeConstants.kBottomInverted);
    // intakeMotorTop.setSmartCurrentLimit(
    // IntakeConstants.stallCurentLimit,
    // IntakeConstants.freeCurentLimit
    // );
    // intakeMotorBottom.setSmartCurrentLimit(
    // IntakeConstants.stallCurentLimit,
    // IntakeConstants.freeCurentLimit
    // );

    MotorTop.burnFlash();
    MotorBottom.burnFlash();

    this.setDefaultCommand(stopCommand());
  }

  public void run() {
    MotorTop.set(IntakeConstants.kSpeedPercent);
    MotorBottom.set(IntakeConstants.kSpeedPercent);
  }
  
  public void reverse(){
    MotorTop.set(-IntakeConstants.kSpeedPercent);
    MotorBottom.set(-IntakeConstants.kSpeedPercent);
  }

  public void stop() {
    MotorTop.set(0);
    MotorBottom.set(0);
  }

  public RunCommand runCommand() {
    return new RunCommand(this::run, this);
  }

  public RunCommand reverseCommand() {
    return new RunCommand(this::reverse, this);
  }

  public RunCommand stopCommand() {
    return new RunCommand(this::stop, this);
  }

  
}
