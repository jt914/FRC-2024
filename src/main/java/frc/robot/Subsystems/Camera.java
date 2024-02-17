package frc.robot.Subsystems;

import java.io.Console;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.*;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private PhotonCamera cam;
    public PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout fieldLayout;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.13, 0.33, 0.35), new Rotation3d(0,35,Math.PI)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.


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
    public double[] getDesiredShoot(double xSpeed, double ySpeed){

        double[] retVal = new double[3];

        /*
        1. use Camera to get pose from hood
        2. calculate optimal arm angle and shooter speed for camera
        might need to do testing
        
        */

        PhotonTrackedTarget shooterTarget = null;
        // PhotonUtils.calculateDistanceToTargetMeters(0, );


        boolean seesShooter = false;
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            if(target.getFiducialId() == 7 || target.getFiducialId() == 4){
                shooterTarget = target;
                seesShooter = true;
            }
        }


 
        if(seesShooter){
            // retVal[0] = shooterTarget.getYaw() * (1 + );
                retVal[0] = -15 + shooterTarget.getYaw() + Math.signum(retVal[0]) *  20* Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
            //18

            
            retVal[1] =
            PhotonUtils.calculateDistanceToTargetMeters(
                    .35,
                    1.778,
                    35 * Math.PI / 180,
                    Units.degreesToRadians(shooterTarget.getPitch()));

            return retVal;
        }

        return null;

        
        


        


        // double[] ret = new double[]{Math.random()* 50, Math.random(), Math.random()};
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