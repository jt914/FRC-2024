package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooterTop;
    private CANSparkMax shooterBot;
    private double targetVelocity;
    private SparkPIDController botPID;
    private SparkPIDController topPID;
    Double botMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Bottom Multiplier").getDouble(0);
    Double topMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Top Multiplier").getDouble(0);


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

        topPID = shooterTop.getPIDController();
        botPID = shooterBot.getPIDController();
        botPID.setP(1);
        topPID.setP(1);
        botPID.setOutputRange(-10000, 10000);

        // botPid = new SparkPIDController(new CanSpark);
        // topPid = new SparkPIDController()

        
        

    }

    public void setSpeed(double botSpeed, double topSpeed) {
        shooterBot.set(botSpeed);
        shooterTop.set(topSpeed);

    }
    public void setVelocity() {
        botPID.setReference(-0.7 * 5676, CANSparkMax.ControlType.kVelocity);
        topPID.setReference(-0.6 * 5676, CANSparkMax.ControlType.kVelocity);

        
        //Inputs RPMs into the PID loop rather than voltage, should account for error 
    }

    public void stop() {
        shooterTop.set(0);
        shooterBot.set(0);

    }



    
}
