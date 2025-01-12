// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilities.AutoCommandMap;
import frc.robot.Utilities.LimeLightHelpers;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Documentation on logging (https://frc3748.github.io/Code-Team-Guide/SwerveDrive/logging/)
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // new PowerDistribution(1, ModuleType.kAutomatic); // Enables power distribution logging
    } else {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    AutoCommandMap.mapCmds();
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Testing Lime Light
    // docs (https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/package-summary.html)

    // double tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset from crosshair to target in degrees
    // double ty = LimelightHelpers.getTY(limelightName);  // Vertical offset from crosshair to target in degrees
    // double ta = LimelightHelpers.getTA(limelightName);  // Target area (0% to 100% of image)
    // boolean hasTarget = LimelightHelpers.getTV(limelightName); // Do you have a valid target?

    // double txnc = LimelightHelpers.getTXNC(limelightName);  // Horizontal offset from principal pixel/point to target in degrees
    // double tync = LimelightHelpers.getTYNC(limelightName);  // Vertical  offset from principal pixel/point to target in degrees


    // if !(hasTarget) {return}
    
    // LimelightHelpers.setPipelineIndex("", 0);
    // You can switch pipeline, so we can utilize different piplies for specific purpose

    // Target is present??
  

    // method above is from 2019 + adapted
    // LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    // if (!results.error || !results.error.matches("")) {return} // no error?

    // System.out.println(results.pipelineID);
    // System.out.println(results.botpose);

    // they renamed fiducial to april tag (they the same)
    

    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#target-data
    // docs (https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/LimelightHelpers.LimelightTarget_Fiducial.html)
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
