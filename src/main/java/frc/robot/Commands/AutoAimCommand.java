package frc.robot.Commands;
import com.fasterxml.jackson.databind.deser.std.StdKeyDeserializer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class AutoAimCommand extends Command {
    Drivetrain swerve;
    Arm arm;
    Shooter shooter;
    double[] desired;
    boolean isFinished = false;
    double desiredOffset;
    boolean setDesired;


    public AutoAimCommand(){
        shooter = Constants.shooter;
        arm = Constants.arm;
        addRequirements(Constants.swerve);
        addRequirements(shooter);
        addRequirements(arm);
        isFinished = false;

    }




    @Override
    public void initialize(){
        arm.setDesired(arm.desiredAngle + 5);
        setDesired = false;


        
    }

    @Override
    public void execute(){
        desired = Constants.camera.getDesiredShoot(1,1);

        // SmartDashboard.putBoolean("Is Finished: ", isFinished);
        if(!setDesired && desired != null ){

            if(Constants.camera.getDesiredShoot(1,1)[0] > 0){
                desiredOffset = 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1])));
                setDesired = true;
        }
            else{
                desiredOffset = - 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1])));
                setDesired = true;

            }
        }


        if(desired != null){

                if(Constants.camera.getDesiredShoot(1,1)[0] > 0){
                desiredOffset = 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1])));
                setDesired = true;
                }
                    else{
                        desiredOffset = - 2 * Math.asin(desired[1]/(Math.sqrt(0.169 + desired[1] *desired[1])));
                        setDesired = true;

                }

                if(Math.abs(Math.abs(desiredOffset) - Math.abs(desired[0])) > 1){
                    // SmartDashboard.putNumber("Desired Offset", desiredOffset); 
                    // SmartDashboard.putNumber("curretPos", Math.abs(desired[0]));                
               
                    Constants.swerve.drive(0, 0, -1.5 * Math.signum(desired[0]) );
                }
                else{
                    isFinished = true;

                }

        }

    
        // shooter.setVelocity(desired[1]);

        // arm.moveArm();
        
        // Pose3d transform = Constants.camera.getPose().get().estimatedPose;


        // double desiredAngle =  Math.atan(transform.getX()/transform.getY());
        // if(Math.abs(transform.getRotation().getAngle() - desiredAngle) > 5){
        //     swerve.drive(0, 0, 0.06 * Math.sqrt(Math.abs(desired[0])));
        // }


        

        
    }


    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Constants.swerve.drive(0, 0, 0);
        isFinished = false;
        CommandScheduler.getInstance().schedule(new SwerveCommand());
        //start swerve and arm command again


    }





    
}

