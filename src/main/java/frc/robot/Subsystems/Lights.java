package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class Lights extends SubsystemBase{
    public static AddressableLED strip;
    public static AddressableLEDBuffer ledBuffer;
    int saveH;
    int saveS;
    int saveV;
    Intake intake;
    Shooter shooter;
    Drivetrain swerve;
    

    public Lights()
    {
        strip = new AddressableLED(0);
        int length = 150;
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
            ledBuffer.setHSV(i, 180, 255, 50);
            saveH = 180;
            saveS = 255;
            saveV = 50;
        }
    }
    public void setColorGreen(int start, int end, int v) {
        for(var i = start; i < end; i++) {
            ledBuffer.setHSV(i, 56, 255, 50);
            saveH = 56;
            saveS = 255;
            saveV = 50;
        }
    }
    public void off() {
        for(var i = 30; i < 150; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
    }
    
    public void previous() {
        for(var i = 30; i < 150; i++) {
            ledBuffer.setHSV(i, saveH, saveS, saveV);
        }
    }
    public static boolean hasNote() {
        return Constants.hasNote;
    }   
    public static boolean seesTag() {
        return SwerveCommand.desired != null;
    }
    public static boolean intakeRunning() {
        return Intake.running;
    }
    // public void periodic() {
    //     if(Lights.hasNote() && Lights.seesTag()) {
    //     }
    // }
}
