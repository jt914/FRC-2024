package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class FieldRelativeCommand extends Command{

    


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){

        Constants.m_gyro.calibrateGyro();
      
    
    }

    
    @Override
    public void end(boolean interrupted){
        
    }


   
  
}
    
    
    
    