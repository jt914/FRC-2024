// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import javax.naming.LimitExceededException;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class SwerveModule {
  private static final int kEncoderResolution = 1;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;
  private final CANcoder turnCoder;

  public RelativeEncoder driveEncoder;
  public RelativeEncoder turnEncoder;
  private int driveId;

  // Gains are for example purposes only - must be determined for your own robot!
  public final PIDController drivePIDController = new PIDController(0, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  public final PIDController turnPIDController = new PIDController(0.006, 0.000, 0.00001);
      
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.55493, 2.3014, 0.51488);
  // private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(.5549, 0, 0);
  private double turnEncoder180;
  DigitalInput swerveLimitSwitch;
  CANcoderConfigurator turnConfig;
  MagnetSensorConfigs turnMagConfig;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int CANCoderID) 
  {
    swerveLimitSwitch = new DigitalInput(driveMotorID);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID , MotorType.kBrushless);
    turnCoder = new CANcoder(CANCoderID);

    driveId = driveMotorID;


    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    driveMotor.setInverted(false);
    turnMotor.setInverted(false);
    driveMotor.setOpenLoopRampRate(.7);
    driveMotor.burnFlash();
    turnMotor.burnFlash();
    // unmodified_turningEncoder = m_turningMotor.getEncoder();
    // m_turningEncoder = unmodified_turningEncoder.getPosition()*Math.toRadians((1/360)*(1/12.8));

    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPositionConversionFactor(Constants.driveEncoderPositionConversion);
    turnEncoder.setPositionConversionFactor(Constants.turnEncoderPositionConversion);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turnPIDController.enableContinuousInput(-180,  180);

    turnConfig = turnCoder.getConfigurator();
    turnMagConfig = new MagnetSensorConfigs();
    turnConfig.refresh(turnMagConfig);    
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setModuleState(SwerveModuleState desiredState, int module) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double moduleVelocity = desiredState.speedMetersPerSecond;
    double moduleAngle = desiredState.angle.getDegrees();
    double optimizedModuleOutput[] = glacierOptimized(moduleAngle, getTurn180Angle(), moduleVelocity);

    double driveOutput = optimizedModuleOutput[1];

    // double driveOutput = driveFeedForward.calculate(1, 0) * Math.sqrt(Math.pow(Constants.swerveController.getLeftX(),2) + Math.pow(Constants.swerveController.getLeftY(),2));
    
    driveMotor.setVoltage(driveOutput);




    double turnOutput = MathUtil.clamp(turnPIDController.calculate(getTurn180Angle(), optimizedModuleOutput[0]), -0.4, 0.4);
    turnMotor.set(turnOutput);


  }

  /**
   * 
   * @param desiredModuleAngle Angle passed into the module to move to
   * @param currentModuleAngle Current angle of the module
   * @param moduleVelocity In voltage (relies on motor kv and gear ratio)
   * @return Optimized angle and velocity calculations, in that order
   */
  public double[] glacierOptimized(double desiredModuleAngle, double currentModuleAngle, double moduleVelocity) {
    double optimized[] = new double[2];
      if (Math.abs(desiredModuleAngle - currentModuleAngle) > 90 && Math.abs(desiredModuleAngle) + Math.abs(currentModuleAngle) < 270) {
        if (desiredModuleAngle >= 0) {
            optimized[0] = desiredModuleAngle - 180;
            optimized[1] = moduleVelocity * -1;
        }
        else if (desiredModuleAngle <= 0){
          optimized[0] = desiredModuleAngle + 180;
          optimized[1] = moduleVelocity * -1;
        }
      }
      else {
        optimized[0] = desiredModuleAngle;
        optimized[1] = moduleVelocity;
      }
    return optimized;
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {

    return new SwerveModuleState(
        driveEncoder.getVelocity()/40.5, new Rotation2d(turnEncoder.getPosition()*28.125));
        
      }
  /**
   * Returns the current position of the module.
   *z
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // System.out.println(turnEncoder.getPosition()/12.8 * 2 * Math.PI);
    return new SwerveModulePosition(
        driveEncoder.getPosition()/3.888, new Rotation2d(turnEncoder.getPosition()/12.8 * 2 * Math.PI));

  }
  /**
   * Converts the motor's encoder from an absolute angle to a limited range between -180 to 180
   * 
   * @return Angle of the module converted to a range between -180 degrees and 180 degrees
   */
  public double getTurn180Angle() {
    if (turnEncoder.getPosition()*28.125 > 360) {
      turnEncoder180 = ((turnEncoder.getPosition()*28.125) % 360) - 180;
    }
    else if (turnEncoder.getPosition()*28.125 < 0) {
      turnEncoder180 = ((turnEncoder.getPosition()*28.125) % 360) + 180;
    }
    else {
      turnEncoder180 = (turnEncoder.getPosition()*28.125) - 180;
    }
    return turnEncoder180;
  }

  public void setMagnetOffset(double offset) {
      turnConfig.apply(turnMagConfig.withMagnetOffset(offset));
  }
  public void resetAbsoluteModule() {
    turnEncoder.setPosition(12.8 * turnCoder.getPosition().getValueAsDouble());
  }
}