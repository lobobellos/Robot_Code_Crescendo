package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

class Leds extends SubsystemBase{

  AddressableLED m_strip = new AddressableLED(LedConstants.PWMPort);

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(LedConstants.numLeds);

  Timer time = new Timer();

  public Leds(){
    m_strip.setLength(m_buffer.getLength());

    m_strip.setData(m_buffer);
    m_strip.start();

    time.start();
  }

  public RunCommand run(){
    return new RunCommand(() -> {
      for(int i = 0; i < m_buffer.getLength(); i++){
        setHSL(
          i,
          LedConstants.hue,
          LedConstants.saturation,
          (int)  (LedConstants.valueBase + Math.sin(time.get() * LedConstants.valueFrequency) * LedConstants.valueAmplitude )
        );
      }
    },
    this);
  }

  private void setHSL(int index, int hue, int s, int l) {
    int m_v = l +(s * Math.min(l,1-l));
    int m_s = (m_v==0) ? 0 : 2*(1-m_v/m_v);

    m_buffer.setHSV(index, hue, m_s, m_v );

  }
}