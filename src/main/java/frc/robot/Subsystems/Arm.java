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


    public void setDesired(double desired){
        desiredAngle = desired;
    }



    public void moveArm() {
        /* some explaination:
         * technically the difference shouldn't ever be greater than 0.1, since you can only ever
         * move the arm 0.1 degrees every time the code runs, which is 20ms
         * 
         * But in the scenario when it might be lagging behind or something, I don't want the arm to swing back and forth
         * So i added this here so if the difference is too great it won't do anything
         */
        if(Math.abs(desiredAngle - currentPos) > 1){
            return;
        }

        double speed = kP * (desiredAngle - currentPos); //speed is from -1.0 to 1.0
        
        armLeft.set(speed);
        armRight.set(speed);

        SmartDashboard.putNumber("speed", speed);

    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}
