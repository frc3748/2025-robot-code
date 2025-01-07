package frc.robot.Utilities;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;


public class AutoCommandMap {
    public static void mapCmds(){
        NamedCommands.registerCommand("CMD0", new PrintCommand("++++ Performed CMD0 ++++"));
        NamedCommands.registerCommand("CMD1", new PrintCommand("++++ Performed CMD1 ++++"));
        NamedCommands.registerCommand("CMD2", new PrintCommand("++++ Performed CMD2 ++++"));
        NamedCommands.registerCommand("CMD3", new PrintCommand("++++ Performed CMD3 ++++"));
        NamedCommands.registerCommand("CMD4", new PrintCommand("++++ Performed CMD4 ++++"));
        NamedCommands.registerCommand("CMD5", new PrintCommand("++++ Performed CMD5 ++++"));
        NamedCommands.registerCommand("CMD6", new PrintCommand("++++ Performed CMD6 ++++"));
        NamedCommands.registerCommand("CMD7", new PrintCommand("++++ Performed CMD7 ++++"));
        NamedCommands.registerCommand("CMD8", new PrintCommand("++++ Performed CMD8 ++++"));
        NamedCommands.registerCommand("CMD9", new PrintCommand("++++ Performed CMD9 ++++"));

        NamedCommands.registerCommand("testarm", new InstantCommand(() -> SmartDashboard.putNumber("ArmSetpoint", 10)));

        // NamedCommands.registerCommand("INTAKE", new IntakeAuto(shintake));


    }
}
