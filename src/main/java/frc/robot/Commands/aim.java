package frc.robot.Commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Camera;


public class aim extends CommandBase{

    public aim(){
        //maybe depending on which target (apriltag) it sees, it will adjust aim code so only need 1 button
    }


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){
        Camera.LimeLight vision = new Camera.LimeLight();

        for (double i : vision.getTids()) {
            System.out.println(i);
        }
    }

    
    @Override
    public void end(boolean interrupted){
        
    }




}
