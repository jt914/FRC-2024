package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase{

    private CANSparkMax intake;
    public boolean running;

    public Intake() {
        intake = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setIdleMode(IdleMode.kCoast);
        intake.enableVoltageCompensation(11);
        intake.burnFlash();
    }

    public void run(double speed) {
        intake.set(speed);
        // intake.set(0.4);
        running = true;
    }
    public void runSlow() {
        // intake.set(0.85);
        intake.set(0.25);
        running = true;
    }


    public void reverse() {
        intake.set(-0.25);
        running = true;

    }

    public void stop() {
        intake.set(0);
        running = false;

    }



    
}
