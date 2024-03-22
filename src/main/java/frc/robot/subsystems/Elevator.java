  package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    motorTop.burnFlash();
    motorBottom.burnFlash();

    this.setDefaultCommand(stopCommand());
  }

  public void run() {
    motorTop.set(ElevatorConstants.kSpeedPercent);
    motorBottom.set(ElevatorConstants.kSpeedPercent);
  }
  
  public void reverse(){
    motorTop.set(-ElevatorConstants.kSpeedPercent);
    motorBottom.set(-ElevatorConstants.kSpeedPercent);
  }

  public void stop() {
    motorTop.set(0);
    motorBottom.set(0);
  }

  public Command runCommand() {
    return Commands.run(this::run, this);
  }

  public Command reverseCommand(){
    return Commands.run(this::reverse,this);
  }

  public Command stopCommand() {
    return Commands.run(this::stop, this);
  }
}
