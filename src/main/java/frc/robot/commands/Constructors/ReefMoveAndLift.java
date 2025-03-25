// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Constructors;

import java.util.List;

import javax.sound.midi.SysexMessage;

import com.fasterxml.jackson.databind.ser.std.ToEmptyObjectSerializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Ladder.LadderMoveToPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.Constants;


/** Add your docs here. */

public class ReefMoveAndLift {
    private PathMaker pathMaker = new PathMaker();
    private Command returnCommands;
    private final Transform2d tagTorobotLeftStalkPosiion = new Transform2d(-1,0.17, new Rotation2d(0));
    private final Transform2d tagTorobotRightStalkPosiion = new Transform2d(-1,-0.17, new Rotation2d(0));
    /**
     * @return Command to be scheduled
     * @param Stalk Left=1, Right=2
     * @param Level Bottom=1, Middle=2, Top=3
     * @param Ladder m_ladder
     * @param Drive m_drive
     * @param robotToTag Transform2d from robot pose to tag pose
     */
    public Command createCommand(int stalk, int level, LadderSubsystem ladder, DriveSubsystem drive, Intake intake, Transform2d robotToTag){
        if(robotToTag != null){
            //Left Stalk
            //L2
            if(stalk == 1 && level == 1){
                var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.bottomStalkPosition))
                .andThen(new IntakeOut(intake));
            }
            //L3
            else if(stalk == 1 && level == 2){
                var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.middleStalkPosition))
                .andThen(new IntakeOut(intake));
            }
            //L4
            else if(stalk == 1 && level == 3){
                var robotToPosition = robotToTag.plus(tagTorobotLeftStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.topStalkPosition))
                .andThen(new IntakeOut(intake));
            }

            //Right Stalk
            //L2
            else if(stalk == 2 && level == 1){
                var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.bottomStalkPosition))
                .andThen(new IntakeOut(intake));
            } 
            //L3
            else if(stalk == 2 && level == 2){
                var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.middleStalkPosition))
                .andThen(new IntakeOut(intake));
            }
            //L4
            else if(stalk == 2 && level == 3){
                var robotToPosition = robotToTag.plus(tagTorobotRightStalkPosiion);
                returnCommands = pathMaker.createPath(drive, new Pose2d().plus(robotToPosition), List.of(), false)
                .alongWith(new LadderMoveToPosition(ladder, Constants.LadderConstants.topStalkPosition))
                .andThen(new IntakeOut(intake));
            }
            return new Command(){};
        }
        SmartDashboard.putString("Status", "No Tag Found");
        return new Command(){};
    }


}
