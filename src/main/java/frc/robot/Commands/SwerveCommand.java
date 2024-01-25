package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class SwerveCommand extends Command{

    private boolean fieldRelative = false;
    private boolean driving = true;


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){
        // Constants.m_swerve.updateOdometry();

        if (Constants.swerveController.getBackButtonPressed()) {
          fieldRelative = !fieldRelative;
        }
        if(driving){
            SmartDashboard.putBoolean("fieldRelative", fieldRelative);
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


    public void driveWithJoystick(boolean fieldRelative) {
      double xSpeed = 0;
      double ySpeed = 0;
      double yaw = 0;
      boolean aboveDeadband = false;
   
      /* Converts the cartesian X and Y axes of the left stick of the swerve controller to the polar coordinate "r" */
      /* This is the distance from (0, 0) to the cartesian coordinate of the left stick output */
      /* If this absolute "r" value is less than the left stick deadband (set in Constants), the xSpeed and ySpeed are not calculated */
      if (Math.abs(Math.sqrt(Math.pow(Constants.swerveController.getLeftX(), 2) + Math.pow(Constants.swerveController.getLeftY(), 2))) > Constants.swerveControllerLeftStickDeadband) {
        aboveDeadband = true;
        if (fieldRelative == true) {
          ySpeed = (Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (Constants.swerveController.getLeftX()) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
          xSpeed = -1 * Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) + (Constants.swerveController.getLeftX()) * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;
        }
        else {
          ySpeed = -1 * Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftX()) * Drivetrain.kMaxVoltage;
          xSpeed = -1 * Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY()) * Drivetrain.kMaxVoltage;
        }
      }
      else {
        if (aboveDeadband == true) {
          aboveDeadband = false;
          ySpeed = 0.1;
        }
        ySpeed = 0;
        xSpeed = 0;
      }
      yaw = -1 * Constants.m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.swerveController.getRightX(), Constants.swerveControllerRightXDeadband)) * Drivetrain.kMaxAngularSpeed;
  
      // SmartDashboard.putNumber("xSpeed ", xSpeed);
      // SmartDashboard.putNumber("ySpeed ", ySpeed);
      // SmartDashboard.putNumber("yaw ", yaw);
      // SmartDashboard.putNumber("gyro angle ", Constants.m_gyro.getTotalAngleDegrees());
      SmartDashboard.putNumber("LeftX", Constants.swerveController.getLeftX());
      SmartDashboard.putNumber("LeftY", Constants.swerveController.getLeftY());

      // Constants.m_swerve.drive(xSpeed, ySpeed, yaw);
    }
  
}
    
    
    
    