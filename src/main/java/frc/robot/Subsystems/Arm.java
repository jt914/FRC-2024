package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase{

    private double currentPos;
    private CANSparkMax armLeft,armRight;
    // private DutyCycleEncoder armEnc;
    private RelativeEncoder armEnc;

    private double kP, kI, kD;

    public Arm() {
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kCoast);
        armLeft.enableVoltageCompensation(11);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kCoast);
        armRight.enableVoltageCompensation(11);
        armRight.burnFlash();

        // armEnc = new DutyCycleEncoder(1); //idk what goes here
        armEnc = armLeft.getEncoder(); //idk what goes here

        kP = 0.01;
        kI = 0;
        kD = 0;

    }

    /**
     * 
     * @return updates and returns updated position of arm
     */
    public double updateAngle(){
        currentPos = armEnc.getPosition(); //need to put some conversion factor here (??)
        return currentPos;
    }

    /**
     * 
     * @param desiredAngle desired arm angle
     */

    public void setAngle(double desiredAngle) {
        double power = kP * (desiredAngle - currentPos);
        if(Math.abs(desiredAngle - currentPos) > 2){
            armLeft.set(power);
            armRight.set(power);

        }
    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}
