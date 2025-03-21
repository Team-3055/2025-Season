// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  static AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  boolean cameraConnected = true; 
  
  static Transform3d frontCameraToRobot = new Transform3d(new Translation3d(-0.2,0,-0.05), new Rotation3d(0,0,0));
  static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCameraToRobot);
  static PhotonPipelineResult frontCameraResult;
  static PhotonPipelineResult backCameraResult;

  PhotonCamera frontCamera = new PhotonCamera("FrontCam");
  PhotonCamera backCamera = new PhotonCamera("BackCam");

  public VisionSubsystem(){
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    /*try{
      frontCamera = new PhotonCamera("FrontCam");
      backCamera = new PhotonCamera("BackCam");
    } catch(ArithmeticException exception){
      cameraConnected = false;
    }*/
  }
  public void updateCamera(){
    if(frontCamera.isConnected()){
      List<PhotonPipelineResult> cameraResults = frontCamera.getAllUnreadResults();
      frontCameraResult = cameraResults.get(cameraResults.size() - 1);
    }
    if(backCamera.isConnected()){
      List<PhotonPipelineResult> cameraResults = backCamera.getAllUnreadResults();
      backCameraResult = cameraResults.get(cameraResults.size() - 1);
    }
  }
  public EstimatedRobotPose getEstimatedGlobalPose() {
    if(frontCamera.isConnected()){
      var estimatedRobotPose = poseEstimator.update(frontCameraResult);
      if(estimatedRobotPose.isPresent()){
        poseEstimator.setLastPose(estimatedRobotPose.get().estimatedPose);
        return estimatedRobotPose.get();
      }
    }
    return null;
  }
  public Pose2d getEstimatedLocalPose(){
    if(backCamera.isConnected()){
      List<PhotonTrackedTarget> targetResults = frontCameraResult.getTargets();
      if(!targetResults.isEmpty()){
        Transform3d targetToRobot = targetResults.get(0).getBestCameraToTarget().inverse().plus(frontCameraToRobot);
        return new Pose2d(targetToRobot.getTranslation().toTranslation2d(), targetToRobot.getRotation().toRotation2d());
      }
    }
    return null;
  }
}
