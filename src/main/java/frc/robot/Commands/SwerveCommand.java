package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class SwerveCommand extends Command{

    int autoAimLoop;
    boolean autoAim = false;
    public static double[] desired;
    boolean isFinished = false;
    double desiredOffset;
    boolean setDesired;
    PIDController aimController = new PIDController(.19, 0.000001, 0);
    InterpolatingDoubleTreeMap tm = new InterpolatingDoubleTreeMap();
    double prevXSpeed, prevYSpeed;
    long prevTime, currTime;
    double a;


    public SwerveCommand(){
      if(Constants.swerve != null) {
        addRequirements(Constants.swerve);
      }
      a = 0.1;
    }

    @Override
    public void initialize(){
      tm.put(3.74, 31.5);
      tm.put(2.4, 26.5);
      tm.put(1.3, 15.5);
      prevTime = System.currentTimeMillis();
      currTime = System.currentTimeMillis();

    }

    @Override
    public void execute(){
      SmartDashboard.putBoolean("field", Constants.fieldRelative);

        if(Constants.swerveController.back().getAsBoolean()){
          Constants.fieldRelative = true;
          Constants.m_gyro.calibrateGyro();

        }

        SmartDashboard.putNumber("Gyrorr", Constants.m_gyro.getTotalAngleDegrees());
        driveWithJoystick(Constants.fieldRelative, Constants.autoAim);

      }
    
    @Override
    public void end(boolean interrupted){
        
    }

    public void driveWithJoystick(boolean fieldRelative, boolean autoAim) {


      double xSpeed = 0;
      double ySpeed = 0;
      double yaw = 0;
      boolean aboveDeadband = false;
   
      if (Math.abs(Math.sqrt(Math.pow(Constants.swerveController.getLeftX(), 2) + Math.pow(Constants.swerveController.getLeftY(), 2))) > Constants.swerveControllerLeftStickDeadband) {
        aboveDeadband = true;
        if (fieldRelative == true) {

          double yController = a * Math.pow(Constants.swerveController.getLeftY(),3) + (1-a) * Constants.swerveController.getLeftY();
          double xController = a * Math.pow(Constants.swerveController.getLeftX(),3) + (1-a) * Constants.swerveController.getLeftX();

          SmartDashboard.putNumber("yController", yController);
          SmartDashboard.putNumber("xController", xController);
          SmartDashboard.putNumber("CURRENTX", Constants.swerve.poseEstimator.getEstimatedPosition().getX());
          SmartDashboard.putNumber("CURRENTY", Constants.swerve.poseEstimator.getEstimatedPosition().getY());

          ySpeed = Constants.m_yspeedLimiter.calculate(yController * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (xController) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;
          xSpeed = Constants.m_xspeedLimiter.calculate(yController * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) + (xController) * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;
          // System.out.println(ySpeed);

        }
        else {
          ySpeed = 0;
          xSpeed = 0;
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
      yaw = Constants.m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.swerveController.getRightX(), Constants.swerveControllerRightXDeadband)) * Drivetrain.kMaxAngularSpeed;

      if(autoAim){
          Constants.shooter.setVelocity();
          desired = Constants.camera.getDesiredShoot(0.7 * -1 * ySpeed);
          if(desired != null && desired[0] != 0){
            yaw = -1 * aimController.calculate(desired[0], -1);
            Constants.arm.setDesired(tm.get(desired[2]) + (xSpeed * 1.05));
            xSpeed = 0.6 * xSpeed;
            ySpeed = 0.6 * ySpeed;    

        }

      }
      if(!autoAim){
        Constants.shooter.stop();
      }

    
      if(Constants.swerve != null) {
        Constants.swerve.drive(ySpeed, xSpeed, yaw);
      }
      
    }
}
    
    
    
    