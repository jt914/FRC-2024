package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class SwerveCommand extends Command{

    private boolean driving = true;
    int autoAimLoop;
    boolean autoAim = false;
    double[] desired;
    boolean isFinished = false;
    double desiredOffset;
    boolean setDesired;
    PIDController aimController = new PIDController(.4, 0.001, .1);



    public SwerveCommand(){
      addRequirements(Constants.swerve);
    }


    @Override
    public void initialize(){

        


    }

    
    @Override
    public void execute(){
        // Constants.m_swerve.updateOdometry();

        if(Constants.alternateController.leftTrigger().getAsBoolean()){
          Constants.arm.forward();
        }
        else if(Constants.alternateController.rightTrigger().getAsBoolean()){
          Constants.arm.backward();
        }
        else{
          Constants.arm.stall();
        }

        if(Constants.alternateController.leftBumper().getAsBoolean()){
          Constants.climber.forward();
        }
        else if(Constants.alternateController.rightBumper().getAsBoolean()){
          Constants.climber.backward();
        }
        else{
          Constants.climber.stall();
        }




        if(Constants.swerveController.back().getAsBoolean()){
          Constants.fieldRelative = !Constants.fieldRelative;
        }

        if(Constants.swerveController.b().getAsBoolean()){
          autoAim = !autoAim;
        }

        if(driving){
            SmartDashboard.putBoolean("fieldRelative", Constants.fieldRelative);
            driveWithJoystick(Constants.fieldRelative, autoAim);
        }

        
        if(Constants.alternateController.back().getAsBoolean()){
          Constants.m_gyro.calibrateGyro();
        }
        
        SmartDashboard.putBoolean("Field Relative", Constants.fieldRelative);
      }
    
    

    
    @Override
    public void end(boolean interrupted){
        
    }






    public void driveWithJoystick(boolean fieldRelative, boolean autoAim) {

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
          ySpeed = -1* (Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (Constants.swerveController.getLeftX()) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
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
  

      if(autoAim){
        desired = Constants.camera.getDesiredShoot(xSpeed,ySpeed);

        if(desired == null || desired[0] == 0){
          Constants.arm.setDesired(Constants.arm.armEnc.getDistance());
        }

        if(desired != null && desired[0] != 0){
            Constants.arm.setDesired(Math.min(45,desired[1] * 20 / 2.75)); //ARM CONSTANT
            
            Constants.arm.moveArm(); 

            yaw = aimController.calculate(desired[0], 0);
        }
      }

      Constants.swerve.drive(xSpeed, ySpeed, yaw);
    }
  
}
    
    
    
    