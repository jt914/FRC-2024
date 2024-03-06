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
    public PIDController controller = new PIDController(.38, 0.001, .01);

    // public PIDController controller = new PIDController(.2, .13, 0);
    private final ArmFeedforward feedforward  = new ArmFeedforward(0, .22, 0);
    public DigitalInput armSwitchBot;

    public Arm() {
        armLeft = new CANSparkMax(Constants.armLeftID, MotorType.kBrushless);
        armLeft.restoreFactoryDefaults();
        armLeft.setIdleMode(IdleMode.kBrake);
        armLeft.enableVoltageCompensation(11);
        armLeft.setSmartCurrentLimit(5);
        armLeft.burnFlash();

        armRight = new CANSparkMax(Constants.armRightID, MotorType.kBrushless);
        armRight.restoreFactoryDefaults();
        armRight.setIdleMode(IdleMode.kBrake);
        armRight.enableVoltageCompensation(11);
        armRight.setSmartCurrentLimit(5);
        armRight.setInverted(true);

        armRight.burnFlash();

        // armSwitchBot = new DigitalInput(8);
        armEnc = new DutyCycleEncoder(8);
        // armEnc.setPositionOffset(.9);
        // armEnc.setDistancePerRotation(-360);

        kP = .05;
        kI = 0;
        kD = 0;
    }

    public void setDesired(double desired){
        desiredAngle = desired;
        System.out.println(desiredAngle);

    }

    public void moveArm() {
        desiredAngle = MathUtil.clamp(this.desiredAngle, 0,120);

        // SmartDashboard.putNumber("Brake", armLeft.getBusVoltage()*armLeft.getAppliedOutput());
        
        // double k = Math.signum(desiredAngle - getAngle());
        // if(Math.abs(desiredAngle-getAngle()) < 2);
        // {
        //     k = 0;
        // }
        // SmartDashboard.putNumber("pid", controller.calculate(((1 - armEnc.getAbsolutePosition()) * 360),desiredAngle) );
        // SmartDashboard.putNumber("ff", feedforward.calculate(desiredAngle* Math.PI / 180, Math.PI/10));
        // SmartDashboard.putNumber("k", k);
        // // if(armSwitchBot.get()) {
        // //     if(armLeft.getAppliedOutput() < 0) {
        // //         // arm voltage is negative
        // //     }
        // // }
        SmartDashboard.putNumber("voltage", controller.calculate(getAngle(), desiredAngle));
        SmartDashboard.putNumber("currentAgnleee", getAngle());
        SmartDashboard.putNumber("desiredAngleee", desiredAngle);
        armRight.setVoltage(controller.calculate(getAngle(), desiredAngle) + (feedforward.calculate(getAngle() * Math.PI / 180, 0)));
        armLeft.setVoltage(controller.calculate(getAngle(), desiredAngle) + (feedforward.calculate(getAngle() * Math.PI / 180, 0)));
        SmartDashboard.putData("pidControl", controller);
    }
    public double getAngle() {
        return ((1 - armEnc.getAbsolutePosition()) * 360) -19.6;
    }
    
    public void stop() {
        armLeft.set(0);
        armRight.set(0);

    }
}