package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase{

    private CANSparkMax shooterTop;
    private CANSparkMax shooterBot;

    public Shooter() {
        shooterTop = new CANSparkMax(Constants.shooterTopID, MotorType.kBrushless);
        shooterTop.restoreFactoryDefaults();
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTop.enableVoltageCompensation(11);
        shooterTop.burnFlash();

        shooterBot = new CANSparkMax(Constants.shooterBotID, MotorType.kBrushless);
        shooterBot.restoreFactoryDefaults();
        shooterBot.setIdleMode(IdleMode.kCoast);
        shooterBot.enableVoltageCompensation(11);
        shooterBot.burnFlash();
    }

    public void setSpeed(double botSpeed, double topSpeed) {
        shooterBot.set(botSpeed);
        shooterTop.set(topSpeed);

    }

    public void stop() {
        shooterTop.set(0);
        shooterBot.set(0);

    }



    
}
