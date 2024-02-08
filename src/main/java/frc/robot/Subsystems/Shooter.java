package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
    public SparkPIDController controller;
    Double botMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Bottom Multiplier").getDouble(0);
    Double topMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Top Multiplier").getDouble(0);


    public Shooter() {
        shooterTop = new CANSparkMax(Constants.shooterTopID, MotorType.kBrushless);
        shooterTop.restoreFactoryDefaults();
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTop.enableVoltageCompensation(11);
        shooterTop.burnFlash();

        shooterBot = shooterTop;

        controller = shooterTop.getPIDController();

        double kP = 0.001;
        double kI = 0.00001;
        double kD = 0; 
        double kIz = 0; 
        double kFF = 0.000015; 
        double kMaxOutput = 1; 
        double kMinOutput = -1;
        double maxRPM = 5700;
    
        // set PID coefficients
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setIZone(kIz);
        controller.setFF(kFF);
        controller.setOutputRange(kMinOutput, kMaxOutput);
    

        // shooterBot = new CANSparkMax(Constants.shooterBotID, MotorType.kBrushless);
        // shooterBot.restoreFactoryDefaults();
        // shooterBot.setIdleMode(IdleMode.kCoast);
        // shooterBot.enableVoltageCompensation(11);
        // shooterBot.burnFlash();

        


        // botPid = new SparkPIDController(new CanSpark);
        // topPid = new SparkPIDController()

        
        

    }

    public void setSpeed(double botSpeed, double topSpeed) {
        shooterBot.set(botSpeed);
        shooterTop.set(topSpeed);

    }
    public void setVelocity() {
        System.out.println(shooterTop.getEncoder().getVelocity());
        controller.setReference(600, ControlType.kVelocity);
        

        
        //Inputs RPMs into the PID loop rather than voltage, should account for error 
    }

    public void stop() {
        shooterTop.set(0);
        shooterBot.set(0);

    }



    
}
