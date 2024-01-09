package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private PhotonCamera cam;

    public Camera(){
        cam = new PhotonCamera("photonvision");
        
    }

    public PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
        
    }

    public void end(){
        cam.close();
    }

}
