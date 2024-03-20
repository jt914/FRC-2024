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


public class Winch extends SubsystemBase{

    private CANSparkMax winchLeft,winchRight;
    public RelativeEncoder winchLeftEncoder, winchRightEncoder;

    public Winch() {
        winchLeft = new CANSparkMax(Constants.winchLeftID, MotorType.kBrushless);
        winchLeft.restoreFactoryDefaults();
        winchLeft.setIdleMode(IdleMode.kBrake);
        winchLeft.enableVoltageCompensation(11);
        winchLeft.setSmartCurrentLimit(1);

        winchRight = new CANSparkMax(Constants.winchRightID, MotorType.kBrushless);
        winchRight.restoreFactoryDefaults();
        winchRight.setIdleMode(IdleMode.kBrake);
        winchRight.enableVoltageCompensation(11);
        winchRight.setInverted(true);
        winchRight.setSmartCurrentLimit(1);

        winchRight.burnFlash();
        winchLeft.burnFlash();

        winchLeftEncoder = winchLeft.getEncoder();
        winchRightEncoder = winchRight.getEncoder();
    }
    public void moveOut() {
        if(winchRightEncoder.getPosition() < 100 && winchLeftEncoder.getPosition() < 100) {
            winchLeft.set(-0.1);
            winchRight.set(-0.1);
        }

    }
    public void moveRightOut(){
        if(winchRightEncoder.getPosition() < 85) {
            winchRight.set(0.1);
        }

    }
    public void moveLeftOut(){
        if(winchLeftEncoder.getPosition() < 72) {
            winchLeft.set(0.1);
        }

    }
    public void moveLeftIn() {
        if(winchLeftEncoder.getPosition() > 5) {
            winchLeft.set(-0.1);
        }

    }
    public void moveRightIn() {
        if(winchRightEncoder.getPosition() > 5) {
            winchRight.set(-0.1);
        }
    }

    public void stop() {
        winchLeft.set(0);
        winchRight.set(0);

    }
}