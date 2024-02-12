package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  
  private static PneumaticHub pneumaticHub = new PneumaticHub();

  private DoubleSolenoid shooterSolenoid = pneumaticHub.makeDoubleSolenoid(
    PneumaticsConstants.shooterDoubleSolenoid[0],
    PneumaticsConstants.shooterDoubleSolenoid[1]
  );

  public Pneumatics(){
    setCompressorEnabled(true);
    setShooterPiston(false);
  }

  public boolean compressorEnabled(){
    return pneumaticHub.getCompressor();
  }

  public void setCompressorEnabled(boolean enable){
    if(enable){
      pneumaticHub.enableCompressorAnalog(0, PneumaticsConstants.maxPressure);
    }else{
      pneumaticHub.disableCompressor();
    }
  }

  public void toggleCompressor(){
    setCompressorEnabled(!compressorEnabled());
  }

  public void setShooterPiston(boolean state){
    shooterSolenoid.set( state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
  }

  public void toggleShooterPiston(){
    shooterSolenoid.toggle();
  }

}
