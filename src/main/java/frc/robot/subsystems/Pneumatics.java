package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  private PneumaticsControlModule controller = new PneumaticsControlModule(11);
  private Compressor compressor = new Compressor(11,PneumaticsModuleType.CTREPCM);

  public Pneumatics(){
    enableCompressor();
    setShooterPistonDown();

    controller.setOneShotDuration(0, 250);
    controller.setOneShotDuration(1, 250);
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

  public InstantCommand enableCompressorCommand(){
    return new InstantCommand(this::enableCompressor);
  }

  public InstantCommand disableCompressorCommand(){
    return new InstantCommand(this::disableCompressor);
  }

  public void setShooterPistonUp(){
    controller.fireOneShot(PneumaticsConstants.pistonUpPort);
  }

  public void setShooterPistonDown(){
    controller.fireOneShot(PneumaticsConstants.pistonDownPort);
  }

  public InstantCommand setShooterPistonUpCommand(){
    return new InstantCommand(this::setShooterPistonUp, this);
  }

  public InstantCommand setShooterPistonDownCommand(){
    return new InstantCommand(this::setShooterPistonDown, this);
  }

  public void periodic(){
    //SmartDashboard.putBoolean("compressor enabled", compressorEnabled());
    SmartDashboard.putNumber("compressor current", compressor.getCurrent());
  }
}
