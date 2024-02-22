package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase{

    private CANSparkMax intake;
    public boolean running;
    public AnalogInput intakeSensor;
    public SparkPIDController intakePidController;

    public Intake() {
        intake = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setIdleMode(IdleMode.kBrake);
        intake.enableVoltageCompensation(11);
        intakePidController = intake.getPIDController();
        intakePidController.setFF(0.00025);
        intakePidController.setP(0.000001);

        intake.burnFlash();
        intakeSensor = new AnalogInput(0);
    }

    public void runFast() {
        intakePidController.setReference(4700, ControlType.kVelocity);
        SmartDashboard.putNumber("intake", intake.getEncoder().getVelocity());
        running = true;
    }
    public void run() {
        intakePidController.setReference(1100, ControlType.kVelocity);
        SmartDashboard.putNumber("intake", intake.getEncoder().getVelocity());
        running = true;
    }

    public void runSlow(){
        intakePidController.setReference(800, ControlType.kVelocity);
        SmartDashboard.putNumber("intake", intake.getEncoder().getVelocity());
        running = true;

    }

    public void reverse() {
        intake.set(-0.2);
        running = true;

    }

    public void stop() {
        intake.set(0);
        running = false;

    }
}