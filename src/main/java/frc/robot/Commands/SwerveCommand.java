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
    boolean setGoal;
    PIDController aimController = new PIDController(.50, 0.000001, 0);



    public SwerveCommand(){
      addRequirements(Constants.swerve);
    }


    @Override
    public void initialize(){



    }

    
    @Override
    public void execute(){
        // Constants.m_swerve.updateOdometry();

        if(Constants.swerveController.back().getAsBoolean()){
          Constants.fieldRelative = !Constants.fieldRelative;
        }

        if(Constants.swerveController.b().getAsBoolean()){
          if (autoAim){
            driving = false;
            autoAim = true;
          }
          else{
            autoAim = false;
            driving = true;
          }
        }
        if(driving){
            SmartDashboard.putBoolean("fieldRelative", Constants.fieldRelative);
            driveWithJoystick(Constants.fieldRelative);
        }

        if(autoAim && autoAimLoop < 100000){
          autoAimLoop++;
          autoAim(Constants.fieldRelative);
        }
        
        if(autoAimLoop >= 100000){
          autoAim = false; 
          driving = true;
        }

        if(Constants.swerveController.povRight().getAsBoolean()){
          Constants.m_gyro.calibrateGyro();
        }
        
        SmartDashboard.putBoolean("Field Relative", Constants.fieldRelative);
      }
    
    

    
    @Override
    public void end(boolean interrupted){
        
    }

    public void autoAim(boolean fieldRelative){

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
          ySpeed = -1 * (Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (Constants.swerveController.getLeftX()) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
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

      desired = Constants.camera.getDesiredShoot(xSpeed,ySpeed);
      // desired = null;
      if(desired == null || desired[0] == 0){
        Constants.swerve.drive(xSpeed,ySpeed,yaw);
      }
    

      if(desired != null){
              System.out.println("running autoaim");
        System.out.println(desired[1] * 20 / 2.75);
        Constants.arm.setGoal(desired[1] * 20 / 2.75); //ARM CONSTANT


        // if(desired[0] > 0){
        // desiredOffset = 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1])));
        // setGoal = true;
        // }
        // else{
        //   desiredOffset = - 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1] + desired[0] )));

        // }


        Constants.swerve.drive(xSpeed, ySpeed, aimController.calculate(desired[0], 0));
        // System.out.println(controller.calculate(desired[0],0));
        // System.out.println(aimController.calculate(desired[0],0));

        // Constants.m_swerve.drive(xSpeed, ySpeed, -1 * Math.abs(Math.cbrt(Math.abs(desired[0]))) * Math.signum(desired[0]) );
      }


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
  
      // SmartDashboard.putNumber("xSpeed ", xSpeed);
      // SmartDashboard.putNumber("ySpeed ", ySpeed);
      // SmartDashboard.putNumber("yaw ", yaw);
      // SmartDashboard.putNumber("gyro angle ", Constants.m_gyro.getTotalAngleDegrees());
      SmartDashboard.putNumber("LeftX", Constants.swerveController.getLeftX());
      SmartDashboard.putNumber("LeftY", Constants.swerveController.getLeftY());
      // System.out.println("xSpeed" + xSpeed + "ySpeed" + ySpeed);

      Constants.swerve.drive(xSpeed, ySpeed, yaw);
    }
  
}
    
    
    
    