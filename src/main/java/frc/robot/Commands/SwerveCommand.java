package frc.robot.Commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class SwerveCommand extends CommandBase{

    private boolean fieldRelative = false;


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){
        Constants.m_swerve.updateOdometry();

        if (Constants.swerveController.getBackButtonPressed()) {
          fieldRelative = !fieldRelative;
        }
        if (Constants.swerveController.getStartButton()) {
            driveWithJoystick(fieldRelative);
        }
        if (Constants.swerveController.getAButtonPressed() == true) {
          Constants.m_gyro.calibrateGyro();
        }
        SmartDashboard.putBoolean("Field Relative", fieldRelative);
    
    }

    
    @Override
    public void end(boolean interrupted){
        
    }

    private void driveWithJoystick(boolean fieldRelative) {
        double xSpeed = 0;
        double ySpeed = 0;
        double yaw = calculateYaw();
    
        if (isAboveDeadband()) {
            if (fieldRelative) {
                ySpeed = calculateFieldRelativeYSpeed();
                xSpeed = calculateFieldRelativeXSpeed();
            } else {
                ySpeed = calculateRobotRelativeYSpeed();
                xSpeed = calculateRobotRelativeXSpeed();
            }
        }
    
        Constants.m_swerve.drive(xSpeed, ySpeed, yaw);
    }

    private boolean isAboveDeadband() {
        double leftX = Constants.swerveController.getLeftX();
        double leftY = Constants.swerveController.getLeftY();
        double r = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        return Math.abs(r) > Constants.swerveControllerLeftStickDeadband;
    }

    private double calculateFieldRelativeYSpeed() {
        return -1 * (Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (-1 * Constants.swerveController.getLeftX()) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
    }
    
    private double calculateFieldRelativeXSpeed() {
        return Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) + (-1 * Constants.swerveController.getLeftX()) * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;

    }
    
    private double calculateRobotRelativeYSpeed() {
        return -1 * Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftX()) * Drivetrain.kMaxVoltage;

    }
    
    private double calculateRobotRelativeXSpeed() {
        return Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY()) * Drivetrain.kMaxVoltage;
    }

    private double calculateYaw() {
        return -1 * Constants.m_rotLimiter.calculate(
            MathUtil.applyDeadband(Constants.swerveController.getRightX(), 
                                   Constants.swerveControllerRightXDeadband)) 
               * Drivetrain.kMaxAngularSpeed;
    }
}
    
    
    
    