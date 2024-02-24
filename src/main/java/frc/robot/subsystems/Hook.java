package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class Hook extends SubsystemBase {

  private final CANSparkMax m_hookMotor = new CANSparkMax(HookConstants.kHookMotorCANID, MotorType.kBrushed);

  private final DigitalInput m_limitSwitchTop = new DigitalInput(HookConstants.limitSwitchTopPort);
  private final DigitalInput m_limitSwitchBottom = new DigitalInput(HookConstants.limitSwitchBottomPort);

  public Hook() {
    setDefaultCommand(new RunCommand(this::stop,this));
  }

  public boolean topSwitchPressed() {
    return !m_limitSwitchTop.get();
  }

  public boolean bottomSwitchPressed() {
    return !m_limitSwitchBottom.get();
  }

  public Command runUntil(BooleanSupplier condition, boolean reverse) {
    return new RunCommand(()->{
      m_hookMotor.set(reverse ? HookConstants.retractSpeed : HookConstants.deploySpeed);
    }, this){
      public boolean isFinished() {
        return condition.getAsBoolean();
      }
    };
  }

  public Command rewindHook(){
    return new SequentialCommandGroup(
      runUntil(this::topSwitchPressed, true),
      runUntil(this::bottomSwitchPressed, true)
    );
  }

  public Command raiseHook(){
    return runUntil(this::topSwitchPressed, false);
  }
  public Command lowerHook(){
    return runUntil(this::bottomSwitchPressed, false);
  }

  public void runRaw(){
      m_hookMotor.set(-HookConstants.retractSpeed);
  }

  public void retractRaw(){
      m_hookMotor.set(HookConstants.retractSpeed);
  }

  public void stop(){
    m_hookMotor.stopMotor();
  }
}

