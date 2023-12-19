package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  private static PowerDistribution pdp;
  private static ShuffleboardTab tab;

  public PDP(int deviceID){
    pdp = new PowerDistribution(deviceID, ModuleType.kCTRE);
    addChild("pdp",pdp);
    tab =Shuffleboard.getTab("PDP");
    tab.addNumber("pdp/voltage", pdp::getVoltage);
    tab.addNumber("pdp/current0", ()->pdp.getCurrent(0));

  }
  
  public RunCommand updatePDP(){
    return new RunCommand(()->{
      
    }, this);
  }


}
