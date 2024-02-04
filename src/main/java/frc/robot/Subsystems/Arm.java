package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase{

    private double currentPos;
    private CANSparkMax armLeft,armRight;
    private DutyCycleEncoder armEnc;
    private SparkLimitSwitch limitSwitch;
    private double kP, kI, kD;
    public double desiredAngle;
    public PIDController armPidController = new PIDController(0.1, 0.000, 0.00001);

    public Arm() {
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setInverted(true);
        armRight.burnFlash();

        // limitSwitch = armRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // limitSwitch.enableLimitSwitch(true);

        // armEnc = new DutyCycleEncoder(1); //idk what goes here
        armEnc = new DutyCycleEncoder(9); //idk what goes here

        kP = 0.01;
        kI = 0;
        kD = 0;

    }

    /**
     * 
     * @return updates and returns updated position of arm
     */
    public double updateAngle(){
        currentPos = armEnc.getDistance(); //need to put some conversion factor here (??)
        return currentPos;
    }

    public void forward(){
        armLeft.set(-0.2);
        armRight.set(-0.2);

    }
    public void backward(){
        armLeft.set(0.2);
        armRight.set(0.2);

    }
    public void climbUp(){
        armLeft.set(armPidController.calculate(armEnc.getDistance(), -Constants.armClimbOffset));
        armRight.set(armPidController.calculate(armEnc.getDistance(), Constants.armClimbOffset));
    }

    /**
     * 
     * @param desiredAngle desired arm angle
     */

    public void setAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
        double voltage = kP * (desiredAngle - currentPos);
        SmartDashboard.putNumber("power", voltage);
        if(Math.abs(desiredAngle - currentPos) > 2){
            armLeft.set(voltage);
            armRight.set(voltage);

        }
    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}
