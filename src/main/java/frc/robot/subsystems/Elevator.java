package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

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

  public Elevator() {
    intakeMotorTop.setInverted(ElevatorConstants.kTopInverted);
    intakeMotorBottom.setInverted(ElevatorConstants.kBottomInverted);

    this.setDefaultCommand(new RunCommand(this::run, this));
  }

  public void run() {
    intakeMotorTop.set(ElevatorConstants.kSpeedPercent);
    intakeMotorBottom.set(ElevatorConstants.kSpeedPercent);
  }

  public void stop() {
    intakeMotorTop.set(0);
    intakeMotorBottom.set(0);
  }
}
