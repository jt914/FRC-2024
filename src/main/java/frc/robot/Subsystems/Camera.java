package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    private PhotonCamera cam;
    public PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout fieldLayout;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    public Camera(){
        cam = new PhotonCamera("photonvision");
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        
    }

    public PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
        
    }

    public Optional<EstimatedRobotPose> getPose() {
        return photonPoseEstimator.update();
    }

    public void end(){
        cam.close();
    }

    //if null the command to get desired shoot should just do nothing
    public double[] getDesiredShoot(){
        /*
        1. use Camera to get pose from hood
        2. calculate optimal arm angle and shooter speed for camera
        might need to do testing
        
        */

        PhotonTrackedTarget shooterTarget;
        // PhotonUtils.calculateDistanceToTargetMeters(0, );

        boolean seesShooter = false;
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            if(target.getFiducialId() == 7 || target.getFiducialId() == 4){
                shooterTarget = target;
                seesShooter = true;
                break;
        }

        if(!seesShooter){
            return null;
        }




        
        }


        


        // double[] ret = new double[]{Math.random()* 50, Math.random(), Math.random()};
        return new double[]{1,0.1,0.1};
        //index 0 is optimal arm angle, index 1 is bot speed, index 2 is top speed, 
        
    }

    public double getDriveOffset(){
        /*
        1. use Camera to get pose from hood
        2. calculate rotation needed to make it
        might need to do testing
        
        */
        return 90;
    }


}
