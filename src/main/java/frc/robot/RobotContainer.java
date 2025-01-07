// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.print.attribute.standard.MediaSize.NA;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveManual;
import frc.robot.Commands.FollowPath;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Commands.AimAtSpeaker;

public class RobotContainer {
  Timer timer = new Timer();
  Timer testTimer = new Timer();

  private Joystick joyDrive = new Joystick(0);
  private Drive drive = new Drive();
  private DriveManual teleopDriveCmd = new DriveManual(drive, joyDrive);

 
  
  public AimAtSpeaker speakerAimer = new AimAtSpeaker(drive);

 

  public RobotContainer() {
    configureBindings();
    
    // Log telemetry to SmartDashboard and Shuffleboard
    drive.log();
    drive.setDefaultCommand(teleopDriveCmd);
    
  
    
    configureNamedCommands();
    // updateAutoSelector must be the last thing called !!!
    FollowPath.updateAutoSelector(drive);
  }

  private void configureBindings() {
    
    // new Trigger(() -> joyDrive.getRawButton(6)).debounce(.1).onTrue(speakerAimer);

  }
  private void configureNamedCommands(){

   
  }

  public Command getAutonomousCommand() {
    return FollowPath.autoChooser.getSelected();
  }

}
