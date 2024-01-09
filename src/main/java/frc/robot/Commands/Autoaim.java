package frc.robot.Commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;


public class Autoaim extends CommandBase{

    public Autoaim(){
    }


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){
        var result = Constants.camera.getLatestResult();

        if(result.hasTargets()){
            List<PhotonTrackedTarget> targets = result.getTargets();
            double dist = targets.get(1).getSkew() - targets.get(2).getSkew();
            //figure out how to find dist between
            if(dist > 0 && dist < 30){
                Robot.m_swerve.drive(0, 0, dist, false);
            }



        }


        
        
    }

    
    @Override
    public void end(boolean interrupted){
        
    }




}
