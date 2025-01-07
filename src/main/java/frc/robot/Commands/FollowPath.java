package frc.robot.Commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.Drive;

import com.pathplanner.lib.auto.AutoBuilder;

public class FollowPath {
    static Timer pathTimer = new Timer();
    public static SendableChooser<Command> autoChooser;

    public static void updateAutoSelector(Drive drive){
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(autoChooser);

    }

}
