package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase{

    public double currentPos;
    private CANSparkMax armLeft,armRight;
    public DutyCycleEncoder armEnc;
    private SparkLimitSwitch limitSwitch;
    private double kP, kI, kD;
    public double desiredAngle;
    public PIDController controller = new PIDController(0.0000013, 0, 0);
    private final ArmFeedforward feedforward  = new ArmFeedforward(0.000009, 0.5, 0);
    public DigitalInput armSwitch;


    public Arm() {
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.burnFlash();

        armRight = armLeft;
        armSwitch = new DigitalInput(9);

        // armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        // armRight.restoreFactoryDefaults();
        // armRight.setIdleMode(IdleMode.kBrake);
        // armRight.enableVoltageCompensation(11);
        // armRight.setInverted(true);
        // armRight.burnFlash();


        // limitSwitch = armRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // limitSwitch.enableLimitSwitch(true);

        // armEnc = new DutyCycleEncoder(1); //idk what goes here

        // armEnc = new DutyCycleEncoder(9); 
        // armEnc.reset();


        kP = .05;
        kI = 0;
        kD = 0;

    }

    /**
     * 
     * @return updates and returns updated position of arm
     */
    public double updateAngle(){
        return currentPos;
    }

    public void stall(){
        armLeft.set(0.013);
        armRight.set(0.013);

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
    }


    public void setDesired(double desired){
        desiredAngle = desired;
    }



    public void moveArm() {





        armLeft.setVoltage(controller.calculate(desiredAngle, currentPos) + feedforward.calculate(desiredAngle, currentPos));
        armRight.setVoltage(controller.calculate(desiredAngle, currentPos) + feedforward.calculate(desiredAngle, currentPos));

        SmartDashboard.putNumber("current Arm Position", currentPos);
        SmartDashboard.putNumber("desired Angle", desiredAngle);





        // if(forward){
        //     speed = - 1 * kP * Math.abs(currentPos - desiredAngle); //speed is from -1.0 to 1.0
        // }
        // else{
        //     speed = 1 * kP * Math.abs(desiredAngle - currentPos); //speed is from -1.0 to 1.0
        // }

        // if(speed > 0.3){
        //     return;
        // }
        
        
        // armLeft.set(speed);
        // armRight.set(speed);

        // SmartDashboard.putNumber("speed", speed);

    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}