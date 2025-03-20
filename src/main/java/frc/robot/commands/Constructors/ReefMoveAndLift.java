// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Constructors;

import java.util.List;

import com.fasterxml.jackson.databind.ser.std.ToEmptyObjectSerializer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LadderSubsystem;


/** Add your docs here. */

public class ReefMoveAndLift {
    private PathMaker pathMaker = new PathMaker();
    private Command returnCommands;
    private final Transform2d tagToLeftStalk = new Transform2d(-1,0.17, new Rotation2d(0));
    private final Transform2d tagToRightStalk = new Transform2d(-1,-0.17, new Rotation2d(0));

    /**
     * @return Command to be scheduled
     * @param Stalk Left=1, Right=2
     * @param Rung Bottom=1, Middle=2, Top=3
     * @param Ladder m_ladder
     * @param Drive m_drive
     * @param robotToTag Transform2d from robot pose to tag pose
     */
    public Command createCommand(int stalk, int rung, LadderSubsystem ladder, DriveSubsystem drive, Transform2d robotToTag){
        switch (stalk) {
            //Left stalk
            case 1:
                pathMaker.createPathGlobal(drive, drive.getPose().plus(robotToTag).plus(tagToLeftStalk), List.of());
                break;

            //Right stalk
            case 2:
                pathMaker.createPathGlobal(drive, drive.getPose().plus(robotToTag).plus(tagToRightStalk), List.of());
                break;
            
            default:
                break;
        }
        return returnCommands;
    }


}
