package frc.robot.Commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class aim extends CommandBase{

    public aim(){
        //maybe depending on which target (apriltag) it sees, it will adjust aim code so only need 1 button
    }


    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){
        var result = Constants.visionCam.getLatestResult();

        if(result.hasTargets()){
            List<PhotonTrackedTarget> targets = result.getTargets();

            //if remote button pressed is b, run speaker aiming
            //speaker aiming is difficult because need to figure out how to calculate aiming even when not facing straight
            //from the speaker
            if(true){

                PhotonTrackedTarget leftSpeaker = targets.get(1);
                PhotonTrackedTarget rightSpeaker = targets.get(1);


                for(PhotonTrackedTarget target : targets){

                    if(target.getFiducialId() == 4 || target.getFiducialId() == 8){
                        leftSpeaker = target;
                    }
                    if(target.getFiducialId() == 3 || target.getFiducialId() == 7){
                        rightSpeaker = target;
                    }

                }
                double dist = leftSpeaker.getSkew() - rightSpeaker.getSkew();
                //figure out how to find dist between
                Robot.m_swerve.drive(0, 0, dist, false);
                
            }



        }


        
        
    }

    
    @Override
    public void end(boolean interrupted){
        
    }




}
