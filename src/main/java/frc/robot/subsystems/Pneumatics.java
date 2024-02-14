package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  
  private PneumaticHub pneumaticHub = new PneumaticHub(PneumaticsConstants.kPneumaticHubPort);

  private Compressor compressor = pneumaticHub.makeCompressor();

  private DoubleSolenoid shooterSolenoid = pneumaticHub.makeDoubleSolenoid(
    PneumaticsConstants.shooterDoubleSolenoid[0],
    PneumaticsConstants.shooterDoubleSolenoid[1]
  );

  public Pneumatics(){
    disableCompressor();
    
  }

  public boolean compressorEnabled(){
    return compressor.isEnabled();
  }

  
  public void enableCompressor(){
    compressor.enableHybrid(100, 110);
  }
  
  public void disableCompressor(){
    compressor.disable();
  }

  

  public void setShooterPiston(boolean state){
    shooterSolenoid.set( state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
  }

  public void toggleShooterPiston(){
    shooterSolenoid.toggle  ();
  }

  public void periodic(){
    SmartDashboard.putBoolean("compressor enabled", compressorEnabled());
    SmartDashboard.putNumber("PSI", pneumaticHub.getPressure(0));
  }

}
