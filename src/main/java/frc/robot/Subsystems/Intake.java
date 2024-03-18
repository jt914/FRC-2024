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
    public static boolean running;
    public AnalogInput intakeSensor;
    public SparkPIDController intakePidController;
    public Lights lights = Constants.lights;

    public Intake() {
        intake = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.enableVoltageCompensation(11);
        intakePidController = intake.getPIDController();
        intakePidController.setFF(0.00025);
        intakePidController.setP(0.000001);
        intake.setIdleMode(IdleMode.kBrake);


        intake.burnFlash();
        intakeSensor = new AnalogInput(0);

    }

    public void runFast() {

        intakePidController.setReference(2000, ControlType.kVelocity);

        running = true;
    }
    public void run() {

        intakePidController.setReference(1000, ControlType.kVelocity);

        running = true;
    }

    public void runSlow(){
        intakePidController.setReference(800, ControlType.kVelocity);
        running = true;

    }

    public void reverse() {
        intake.set(-0.4);
        running = true;

    }

    public boolean hasNote(){
        return intakeSensor.getVoltage() < 0.5;
    }

    public void stop() {
        intake.set(0);
        running = false;

    }
}