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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends ProfiledPIDSubsystem{

    public double currentPos;
    private CANSparkMax armLeft,armRight;
    public DutyCycleEncoder armEnc;
    private SparkLimitSwitch limitSwitch;
    private double kP, kI, kD;
    public double desiredAngle;
    // public PIDController controller = new PIDController(.6, .05, .165);
    public PIDController controller = new PIDController(.1, 0.003, .01);
    // public PIDController controller = new PIDController(.2, .13, 0);
    private final ArmFeedforward feedforward  = new ArmFeedforward(0, .45, .5);
    public DigitalInput armSwitch;
    private static double maxVelocity = 90; // degrees per second
    private static double maxAcceleration = 200; // degrees per seconds^2
    private static ProfiledPIDController profiledPIDController = new ProfiledPIDController(
            .1,
            .003,
            .01,
            new TrapezoidProfile.Constraints(
                maxVelocity, // degrees per second
                maxAcceleration) // degrees per second^2
                , 0.02);
                
    public Arm() {
        super(profiledPIDController);

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

        armSwitch = new DigitalInput(8);
        armEnc = new DutyCycleEncoder(9); 
        armEnc.setPositionOffset(0.513);
        armEnc.setDistancePerRotation(-360);

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
        armLeft.set(-0.2);
        armRight.set(-0.2);
    }

    public void backward(){
        armLeft.set(0.2);
        armRight.set(0.2);
    }

    public void climbUp(){
    }



        
        
    
    public double getMeasurement() {
        return armEnc.getDistance();
        
    }
    public void useOutput(double output, TrapezoidProfile.State desiredSetpoint) {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            maxVelocity, // degrees per second
            maxAcceleration); // degrees per second^2
        
        profiledPIDController.setConstraints(constraints);
        double k = Math.signum(output - armEnc.getDistance());
        if(Math.abs(output-armEnc.getDistance()) < 2)
        {
            k = 0;
        }
        SmartDashboard.putNumber("pid", controller.calculate(armEnc.getDistance(),output) );
        SmartDashboard.putNumber("ff", feedforward.calculate(output* Math.PI / 180, Math.PI/10));
        SmartDashboard.putNumber("k", k);
        // armLeft.setVoltage(controller.calculate(armEnc.getDistance(), output) + MathUtil.clamp(feedforward.calculate(output* Math.PI / 180, k), -2, 2));
        // armRight.setVoltage(controller.calculate(armEnc.getDistance(), output) + MathUtil.clamp(feedforward.calculate(output * Math.PI / 180, k), -2, 2));
        armLeft.setVoltage(controller.calculate(armEnc.getDistance(), output) + (feedforward.calculate(armEnc.getDistance()* Math.PI / 180, k * .3)));
        armRight.setVoltage(controller.calculate(armEnc.getDistance(), output) + (feedforward.calculate(armEnc.getDistance() * Math.PI / 180, k * .3)));
        SmartDashboard.putNumber("Left PID", controller.calculate(armEnc.getDistance(), output));
        SmartDashboard.putNumber("Left FF", MathUtil.clamp(feedforward.calculate(output * Math.PI / 180, k), -2, 2));
        SmartDashboard.putNumber("p", controller.getPositionError() * controller.getP());
        SmartDashboard.putNumber("i", controller.getIZone()* controller.getI());
        SmartDashboard.putNumber("d", controller.getVelocityError()* controller.getD());
        SmartDashboard.putData("pidControl", controller);
    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }



    
}