package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooterTop;
    private CANSparkMax shooterBot;
    public SparkPIDController controllerTop;
    public SparkPIDController controllerBot;

    Double botMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Bottom Multiplier").getDouble(0);
    Double topMultiplier = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("Top Multiplier").getDouble(0);

    public Shooter() {
        shooterTop = new CANSparkMax(Constants.shooterTopID, MotorType.kBrushless);
        shooterTop.restoreFactoryDefaults();
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTop.enableVoltageCompensation(11);
        shooterTop.setInverted(true);

        shooterBot = new CANSparkMax(Constants.shooterBotID, MotorType.kBrushless);
        shooterBot.restoreFactoryDefaults();
        shooterBot.setIdleMode(IdleMode.kCoast);
        shooterBot.setInverted(true);
        shooterBot.enableVoltageCompensation(11);

        controllerTop = shooterTop.getPIDController();
        controllerBot = shooterBot.getPIDController();

        double kP = 0;
        double kI = 0;
        double kD = 0; 
        double kFFtop = 0.0001968;
        double kFFbot = 0.0001968;

        // set PID coefficients
        controllerTop.setP(kP);
        controllerTop.setI(kI);
        controllerTop.setD(kD);
        controllerTop.setFF(kFFtop);

        controllerBot.setP(kP);
        controllerBot.setI(kI);
        controllerBot.setD(kD);
        controllerBot.setFF(kFFbot);

        shooterBot.burnFlash();
        shooterTop.burnFlash();

    }
  /**
   * Converts a percentage to velocity in RPMs.
   *
   * @param percent Percentage in integer form.
   */
    public double toRPM(double percent) {
        return .01 * percent * 5676;
    }


    public void setVelocity() {
        //7 and 9 before
        // shooterBot.set(SmartDashboard.getNumber("shooterBot", 0.79));
        // shooterTop.set(SmartDashboard.getNumber("shooterTop", 0.89));
        // controllerBot.setReference(4484, ControlType.kVelocity);
        // controllerTop.setReference(5052, ControlType.kVelocity);

        // System.out.println("RUNNING");
        // shooterBot.set(0.2);
        // shooterTop.set(0.2);


        controllerTop.setReference(toRPM(70), ControlType.kVelocity);
        controllerBot.setReference(toRPM(80), ControlType.kVelocity);


    }

    public void setLowVelocity(){

        // System.out.println("RUNNING");
        // shooterBot.set(0.2);
        // shooterTop.set(0.2);

        controllerBot.setReference(toRPM(30), ControlType.kVelocity);
        controllerTop.setReference(toRPM(40), ControlType.kVelocity);
    }

    public void stop() {
        shooterTop.set(0);
        shooterBot.set(0);

    }
}