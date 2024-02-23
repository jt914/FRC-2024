package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
    // public PIDController controller = new PIDController(.6, .05, .165);
    public PIDController controller = new PIDController(.35, 0.001, .01);
    // public PIDController controller = new PIDController(.2, .13, 0);
    private final ArmFeedforward feedforward  = new ArmFeedforward(0, .22, 0);
    public DigitalInput armSwitch;

    public Arm() {
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.setSmartCurrentLimit(3);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setSmartCurrentLimit(3);
        armRight.setInverted(true);

        armRight.burnFlash();


        armSwitch = new DigitalInput(8);
        armEnc = new DutyCycleEncoder(9); 
        armEnc.setPositionOffset(.266);
        armEnc.setDistancePerRotation(-360);

        kP = .05;
        kI = 0;
        kD = 0;
    }

    public void setDesired(double desired){
        desiredAngle = desired;
    }

    public void moveArm() {
        
        double k = Math.signum(desiredAngle - armEnc.getDistance());
        if(Math.abs(desiredAngle-armEnc.getDistance()) < 2)
        {
            k = 0;
        }
        armLeft.setVoltage(controller.calculate(armEnc.getDistance(), desiredAngle) + (feedforward.calculate(armEnc.getDistance()* Math.PI / 180, k * .3)));
        armRight.setVoltage(controller.calculate(armEnc.getDistance(), desiredAngle) + (feedforward.calculate(armEnc.getDistance() * Math.PI / 180, k * .3)));
    }
    public double getAngle() {
        return armEnc.getDistance();
    }
    
    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }
}