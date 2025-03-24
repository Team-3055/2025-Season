// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Constructors;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathMaker {
    /**
    * @return Returns a path that the robot will follow
    * @param m_drive Main Drive subsystem
    * @param finalPose Final robot pose
    * @param transitionPoses List of translations the robot will pass through (List.of())
    */
    public static Command createPathGlobal(DriveSubsystem m_drive, Pose2d m_finalPose, List<Translation2d> m_transitionPoses){
        final TrajectoryConfig m_config =
        new TrajectoryConfig(
                AutoConstants.kMaxAutoSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
        
        Trajectory robotTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                m_drive.getPose(),
                //Pass in points the robot should go through          
                List.of(new Translation2d(1,0)),
                //final pose of the robot
                m_finalPose,
                m_config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =

            new SwerveControllerCommand(
                robotTrajectory,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0.25),
                new PIDController(AutoConstants.kPYController, 0, 0.25),
                thetaController,
                m_drive::setModuleStates,
                m_drive);
        Command commandSequence = Commands.sequence(
            swerveControllerCommand);
        return commandSequence;
    }
    /*public static Command createPathLocal(DriveSubsystem m_drive, Transform2d m_finalPoseTransform, List<Translation2d> m_transitionPoses){

        final TrajectoryConfig m_config =
        new TrajectoryConfig(
                AutoConstants.kMaxAutoSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory robotTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                m_drive.getPose(),
                //Pass in points the robot should go through          
                m_transitionPoses,
                //final pose of the robot  
                m_drive.getPose().transformBy(m_finalPoseTransform), 
                     
                m_config);
        System.out.println(robotTrajectory.getStates().get(robotTrajectory.getStates().size() - 1).poseMeters.getX() + " " + robotTrajectory.getStates().get(robotTrajectory.getStates().size() - 1).poseMeters.getY());
        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =

            new SwerveControllerCommand(
                robotTrajectory,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0.5, 0),
                new PIDController(AutoConstants.kPYController, 0.5, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);
        Command commandSequence = Commands.sequence(
            swerveControllerCommand);
        return commandSequence;
    
    }
        */ 
}
