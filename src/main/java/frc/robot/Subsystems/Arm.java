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
    public double desiredAngle;
    
    // public PIDController controller = new PIDController(.38, 0.001, .01);

    private final ArmFeedforward feedforward  = new ArmFeedforward(0, .22, 0);
    public DigitalInput armSwitchBot;
    private static double maxVelocity = 20; 
    private static double maxAcceleration = 50; 
    
    private static ProfiledPIDController profiledPIDController = new ProfiledPIDController(
            .4,
            .005,
            .00008,
            new TrapezoidProfile.Constraints(
                maxVelocity, // degrees per second
                maxAcceleration) // degrees per second^2
                ,.02);

    public Arm() {
        super(profiledPIDController);
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.setSmartCurrentLimit(12);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setSmartCurrentLimit(12);
        armRight.setInverted(true);

        armRight.burnFlash();

        // armSwitchBot = new DigitalInput(8);
        armEnc = new DutyCycleEncoder(8);
        // armEnc.setPositionOffset(.9);
        // armEnc.setDistancePerRotation(-360);
        profiledPIDController.setTolerance(.2);
        setGoal(getMeasurement());
    }

    @Override
    public double getMeasurement() {
        return ((1 - armEnc.getAbsolutePosition()) * 360) -19.6;        
    }

    public void setDesired(double desired){
        desiredAngle = desired;
        desiredAngle = MathUtil.clamp(desiredAngle, 5.5, 120);
    }

    public void moveArm() {
        SmartDashboard.putNumber("Desired Angle: ", desiredAngle);
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State desiredSetpoint) {
            // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            //     maxVelocity, // degrees per second  
            //     maxAcceleration); // degrees per second^2
            
            // profiledPIDController.setConstraints(constraints);

            // armLeft.setVoltage(controller.calculate(armEnc.getDistance(), output) + MathUtil.clamp(feedforward.calculate(output* Math.PI / 180, k), -2, 2));
            // armRight.setVoltage(controller.calculate(armEnc.getDistance(), output) + MathUtil.clamp(feedforward.calculate(output * Math.PI / 180, k), -2, 2));
            output = output + (feedforward.calculate(Math.toRadians(desiredSetpoint.position), Math.toRadians(desiredSetpoint.velocity)));
            output = MathUtil.clamp(output, -5, 5);
            armLeft.setVoltage(output);
            armRight.setVoltage(output);
            SmartDashboard.putNumber("Desired Angle", desiredAngle);
            SmartDashboard.putNumber("Voltage Output", output);  
    }

    public boolean atSetpoint() {
        return profiledPIDController.atSetpoint();
    }

    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }
}