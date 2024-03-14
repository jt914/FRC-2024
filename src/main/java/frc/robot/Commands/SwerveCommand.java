package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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


    public SwerveCommand(){
      if(Constants.swerve != null) {
        addRequirements(Constants.swerve);
      }
    }

    @Override
    public void initialize(){
      tm.put(10.74, 20.0);
      tm.put(7.1, 17.0);
      tm.put(4.3, 11.5);
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
          ySpeed = (Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) - (Constants.swerveController.getLeftX()) * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
          xSpeed = -1 * Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY() * Math.sin(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees())) + (Constants.swerveController.getLeftX()) * Math.cos(Math.toRadians(Constants.m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;
        }
        else {
          xSpeed = -1 * Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftX()) * Drivetrain.kMaxVoltage;
          ySpeed = Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY()) * Drivetrain.kMaxVoltage;
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
          Constants.shooter.setVelocity();
          desired = Constants.camera.getDesiredShoot(0.7 * -1 * ySpeed);
          if(desired != null && desired[0] != 0){
            yaw = aimController.calculate(-1 + desired[0], 0);
            Constants.arm.setDesired(tm.get(desired[1]) + (xSpeed * 1.05));

            xSpeed = 0.4 * xSpeed;
            ySpeed = 0.4 * ySpeed;    
              SmartDashboard.putNumber("wawasdassd", desired[1]);

        }

      }
      if(!autoAim){
        Constants.shooter.stop();
      }

    
      // SmartDashboard.putNumber("prevX", prevXSpeed);
      // SmartDashboard.putNumber("prevY", prevYSpeed);
      // SmartDashboard.putNumber("x", xSpeed);
      // SmartDashboard.putNumber("y", ySpeed);

      //if desired X Speed = 0
      //-1, 0, 1
      //-1, 0, 1

      // if(Math.abs(xSpeed) < 0.1){
      //   xSpeed = prevXSpeed + (-0.1 * Math.signum(prevXSpeed));
      // }
      // else if(Math.abs(prevXSpeed - xSpeed) > 0.6){
      //   xSpeed = prevXSpeed + (0.1 * Math.signum(xSpeed));
      // }
      // if(Math.abs(xSpeed) < 0.07){
      //   xSpeed = 0;
      // }

      // if(Math.abs(ySpeed) < 0.1){
      //   ySpeed = prevYSpeed + (-0.1 * Math.signum(prevYSpeed));
      // }
      // else if(Math.abs(prevYSpeed - ySpeed) > 0.6){
      //   ySpeed = prevYSpeed + (0.1 * Math.signum(ySpeed));
      // }
      // if(Math.abs(xSpeed) < 0.07){
      //   ySpeed = 0;
      // }

      // currTime = System.currentTimeMillis();

      // SmartDashboard.putNumber("xSpeedSwerve", xSpeed);
      // SmartDashboard.putNumber("ySpeedSwerve", ySpeed);

      // SmartDashboard.putNumber("time Diff", currTime - prevTime);
      if(Constants.swerve != null) {
        Constants.swerve.drive(ySpeed, xSpeed, yaw);
      }
      // prevXSpeed = xSpeed;
      // prevYSpeed = ySpeed;

      // prevTime = currTime;
      
    }
}
    
    
    
    