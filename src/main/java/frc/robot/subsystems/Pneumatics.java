package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  
  private PneumaticsControlModule controller = new PneumaticsControlModule(11);
  private Compressor compressor = new Compressor(11,PneumaticsModuleType.CTREPCM);

  private DoubleSolenoid shooterSolenoid = controller.makeDoubleSolenoid(
    PneumaticsConstants.shooterDoubleSolenoid[0],
    PneumaticsConstants.shooterDoubleSolenoid[1]
  );

  public Pneumatics(){
    enableCompressor();
    setShooterPiston(true);
  }

  public boolean compressorEnabled(){
    return compressor.isEnabled();
  }

  
  public void enableCompressor(){
    //compressor.enableAnalog(70, 110);
    compressor.enableDigital();
    System.out.println("enabling compressor");
  }
  
  public void disableCompressor(){
    compressor.disable();
    System.out.println("disabling compressor");
  }

  public void setShooterPiston(boolean state){
    shooterSolenoid.set( state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
  }

  public void toggleShooterPiston(){
    shooterSolenoid.toggle();
  }

  public void periodic(){
    //SmartDashboard.putBoolean("compressor enabled", compressorEnabled());
    SmartDashboard.putNumber("compressor current", compressor.getCurrent());
  }

}
