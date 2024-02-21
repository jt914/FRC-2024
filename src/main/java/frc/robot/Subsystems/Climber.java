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


public class Climber extends SubsystemBase{

    public double currentPos;
    private CANSparkMax armLeft,armRight;
    public DutyCycleEncoder armEnc;
    private SparkLimitSwitch limitSwitch;
    private double kP, kI, kD;
    public double desiredAngle;
    public PIDController controller = new PIDController(.5, 0.001, 0.000001);
    private final ArmFeedforward feedforward  = new ArmFeedforward(0.09, 0.05, 0);


    public Climber() {
        armLeft = new CANSparkMax(Constants.climberLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.climberRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setInverted(true);

        armRight.burnFlash();
        armLeft.burnFlash();


        // armEnc.reset();

        // armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        // armRight.restoreFactoryDefaults();
        // armRight.setIdleMode(IdleMode.kBrake);
        // armRight.enableVoltageCompensation(11);
        // armRight.setInverted(true);
        // armRight.burnFlash();


        // limitSwitch = armRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // limitSwitch.enableLimitSwitch(true);

        // armEnc.reset();

    }

    public void stall(){
        armLeft.set(0.013);
        armRight.set(0.013);
    }

    public void forward(){
        armLeft.set(0.2);
        armRight.set(-0.2);
    }

    public void backward(){
        armLeft.set(-0.2);
        armRight.set(0.2);
    }

    public void climbUp(){
    }

    public void setDesired(double desired){
        desiredAngle = desired;
    }

    public void moveArm() {





        armLeft.setVoltage(controller.calculate(armEnc.getDistance(), desiredAngle) + feedforward.calculate(desiredAngle* Math.PI / 180, Math.PI/10));
        armRight.setVoltage(controller.calculate(armEnc.getDistance(), desiredAngle) + feedforward.calculate(desiredAngle * Math.PI / 180, Math.PI/10));

    }






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

    

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}