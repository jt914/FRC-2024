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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.FlashCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.Swerve.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends LoggedRobot {
    private boolean startedSwerve = false;
    private RobotContainer robot;
    Command autoCommand;
    Lights lights = Constants.lights;

  @Override
  public void robotInit() {

    robot = new RobotContainer();
    Constants.m_gyro.calibrateGyro();
    Constants.arm.setGoal(Constants.arm.desiredAngle);
    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  @Override
  public void robotPeriodic(){
    SmartDashboard.putNumber("CURRENTX", Constants.swerve.poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("CURRENTY", Constants.swerve.poseEstimator.getEstimatedPosition().getY());


    CommandScheduler.getInstance().run();
    Lights.strip.setData(Lights.ledBuffer);

    Constants.swerve.updateOdometry();

  }

  @Override
  public void autonomousInit() {
    autoCommand = robot.getAutonomousCommand();

    if(autoCommand != null){
      autoCommand.schedule();
    }
    Constants.arm.disable();
    Constants.arm.setDesired(Constants.arm.getMeasurement());
    Constants.arm.enable();
    Constants.swerve.resetAllAbsoluteModules();

  }

  @Override
  public void autonomousPeriodic() {
        Constants.arm.setGoal(Constants.arm.desiredAngle);


  }
  
  public void teleopInit() {
    Constants.arm.disable();
    Constants.arm.setDesired(Constants.arm.getMeasurement());
    Constants.arm.enable();
    Constants.swerve.resetAllAbsoluteModules();

    if (autoCommand != null) {
      autoCommand.cancel();
      
    }



  }

  boolean fieldRelative = false;
  @Override
  public void teleopPeriodic() {
    if(Constants.swerve != null) {
      Constants.swerve.updateOdometry();
    }
    Constants.arm.setGoal(Constants.arm.desiredAngle);

  

  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void disabledPeriodic(){
  }
}