// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.opencv.core.TermCriteria;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  static AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  public boolean cameraConnected = true; 
  
  static Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0.305,-0.305,-0.1), new Rotation3d(0,0,0));
  static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCamera);
  static PhotonPipelineResult frontCameraResult;
  static PhotonPipelineResult backCameraResult;
  static List<Integer> reefIds= List.of(6,7,8,9,10,11,17,18,19,20,21,22);
  DoublePublisher xReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("xReefTransform").publish();
  DoublePublisher yReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("yReefTransform").publish();
  DoublePublisher rotReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("rotReefTransform").publish(); 
  PhotonCamera frontCamera = new PhotonCamera("FrontCam");
  PhotonCamera backCamera = new PhotonCamera("BackCam");

  
  public VisionSubsystem(){
    cameraConnected = true;
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
      if(cameraResults.size() != 0){
        frontCameraResult = cameraResults.get(cameraResults.size() - 1);
        //System.out.println(frontCameraResult.getBestTarget().bestCameraToTarget);
      }
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
    if(frontCamera.isConnected()){
      List<PhotonTrackedTarget> targetResults = frontCameraResult.getTargets();
      if(!targetResults.isEmpty()){
        Transform3d targetToRobot = targetResults.get(0).getBestCameraToTarget().inverse().plus(robotToFrontCamera);
        return new Pose2d(targetToRobot.getTranslation().toTranslation2d(), targetToRobot.getRotation().toRotation2d());
      }
    }
    return null;
  }

  public Transform2d getReefTransform(){
    //updateCamera();
    
    if(frontCamera.isConnected()){
      if(frontCameraResult != null){
        List<PhotonTrackedTarget> targetResults = frontCameraResult.getTargets();
        if(targetResults != null){
          if(targetResults.size() != 0){
            if(reefIds.contains(targetResults.get(0).fiducialId) && targetResults.size() != 0){
              Transform3d robotToTarget = robotToFrontCamera.plus(targetResults.get(0).getBestCameraToTarget()).inverse().plus(new Transform3d(new Translation3d(), new Rotation3d(0,0,-Math.PI)));
              rotReefTransform.set(robotToTarget.getRotation().toRotation2d().getDegrees());
              xReefTransform.set(robotToTarget .getX());
              yReefTransform.set(robotToTarget.getY());
              return new Transform2d(new Translation2d(robotToTarget.getTranslation().getY(), -robotToTarget.getTranslation().getX()), robotToTarget.getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI/2)));
            }
          }
        }
      }
    }
    return null;
  }
}
