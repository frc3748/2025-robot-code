// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveManual;
import frc.robot.Commands.FollowPath;
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
