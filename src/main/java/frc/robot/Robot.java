// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.Swerve.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends LoggedRobot {
    Field2d m_field = new Field2d();
    private boolean startedSwerve = false;
    private ShooterCommand currentShoot;
    private IntakeCommand currentIntake;
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

  }

  @Override
  public void robotPeriodic(){
    double ySpeed = -1 * Constants.m_yspeedLimiter.calculate(Constants.swerveController.getLeftX()) * Drivetrain.kMaxVoltage;
    double xSpeed = -1 * Constants.m_xspeedLimiter.calculate(Constants.swerveController.getLeftY()) * Drivetrain.kMaxVoltage;
    double yaw = -1 * Constants.m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.swerveController.getRightX(), Constants.swerveControllerRightXDeadband)) * Drivetrain.kMaxAngularSpeed;

    // SmartDashboard.putNumber("xSpeed ", xSpeed);
    // SmartDashboard.putNumber("ySpeed ", ySpeed);
    // SmartDashboard.putNumber("yaw ", yaw);
    // SmartDashboard.putNumber("gyro angle ", Constants.m_gyro.getTotalAngleDegrees());
    
    Constants.m_swerve.drive(xSpeed, ySpeed, yaw);
  
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    Constants.m_swerve.updateOdometry();
  }
  
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();

    SmartDashboard.putData("Field", m_field);


  }

  boolean fieldRelative = false;
  @Override
  public void teleopPeriodic() {
    if (Constants.swerveController.getStartButtonPressed() && !startedSwerve) {
      startedSwerve = true;
      CommandScheduler.getInstance().schedule(new SwerveCommand());
    }  

    if(Constants.alternateController.getBButtonPressed()){
      if(currentShoot != null){
        CommandScheduler.getInstance().cancel(currentShoot);
      }
      // currentShoot = new ParallelCommandGroup(new AimCommand(), new ShooterCommand());
      currentShoot = new ShooterCommand();

      CommandScheduler.getInstance().schedule(currentShoot);
    }

    if(Constants.alternateController.getAButtonPressed()){
      if(currentIntake != null){
        CommandScheduler.getInstance().cancel(currentIntake);
      }
      currentIntake = new IntakeCommand();
      CommandScheduler.getInstance().schedule(currentIntake);
    }


    CommandScheduler.getInstance().run();
    // m_field.setRobotPose(Constants.m_swerve.m_odometry.getPoseMeters());

  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}