package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax motorTop = new CANSparkMax(
      ElevatorConstants.topMotorID,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motorBottom = new CANSparkMax(
      ElevatorConstants.BottomMotorID,
      CANSparkMax.MotorType.kBrushless);

  public Elevator() {
    motorTop.setInverted(ElevatorConstants.kTopInverted);
    motorBottom.setInverted(ElevatorConstants.kBottomInverted);

    motorTop.setSmartCurrentLimit(
        ElevatorConstants.stallCurentLimit,
        ElevatorConstants.freeCurentLimit);
    motorBottom.setSmartCurrentLimit(
        ElevatorConstants.stallCurentLimit,
        ElevatorConstants.freeCurentLimit);

    this.setDefaultCommand(stopCommand());
  }

  public void run() {
    //motorTop.set(ElevatorConstants.kSpeedPercent);
    //motorBottom.set(ElevatorConstants.kSpeedPercent);
  }

  public void stop() {
    motorTop.set(0);
    motorBottom.set(0);
  }

  public RunCommand runCommand() {
    return new RunCommand(this::run, this);
  }

  public RunCommand stopCommand() {
    return new RunCommand(this::run, this);
  }
}
