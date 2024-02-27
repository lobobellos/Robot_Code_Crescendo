package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class Leds extends SubsystemBase{

  AddressableLED m_strip = new AddressableLED(LedConstants.PWMPort);

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(LedConstants.numLeds);

  Timer time = new Timer();

  public Leds(){
    m_strip.setLength(m_buffer.getLength());

    //m_strip.setData(m_buffer);
    //m_strip.start();


    time.start();
  }

  public void start(){
    m_strip.start();
    time.restart();
  }

  public void stop(){
    m_strip.stop();
    time.stop();
  }

  public void periodic(){
    for(int i = 0; i < m_buffer.getLength(); i++){
      int val = (int)  (LedConstants.valueBase + Math.abs(Math.sin(time.get() * LedConstants.valueFrequency+i)) * LedConstants.valueAmplitude );
      m_buffer.setHSV(
        i,
        LedConstants.hue,
        100,
        val
      );
      //m_buffer.setRGB(i,(int)time.get()%256,0, (int)time.get()%256);
      //System.out.println("LED "+time.get()+" "+val);
    }
    m_strip.setData(m_buffer);
  };
}