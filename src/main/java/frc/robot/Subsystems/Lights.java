package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    AddressableLED strip;
    AddressableLEDBuffer ledBuffer;
    int saveH;
    int saveS;
    int saveV;

    public Lights()
    {
        strip = new AddressableLED(9);
        int length = 75;
        strip.setLength(length);
        ledBuffer = new AddressableLEDBuffer(length);
        strip.start();
    }
    public void setColor(int start, int end, int h, int s, int v) {
        for(var i = start; i < end; i++) {
        ledBuffer.setHSV(i, h, s, v);
        saveH = h;
        saveS = s;
        saveV = v;
        }
    }
    public void setColorRed(int start, int end, int v) {
        for(var i = start; i < end; i++) {
            ledBuffer.setHSV(i, 255, 255, 50);
        }
    }
    public void setColorGreen(int start, int end, int v) {
        for(var i = start; i < end; i++) {
            ledBuffer.setHSV(i, 56, 255, 50);
        }
    }
    public void off() {
        for(var i = 0; i < 75; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
    }
    
    public void previous() {
        for(var i = 0; i < 75; i++) {
            ledBuffer.setHSV(i, saveH, saveS, saveV);
        }
    }   
}
