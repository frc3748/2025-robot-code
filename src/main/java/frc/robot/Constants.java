package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities.Config.DynaPerfConf;

public class Constants {
    public static class Module{

        public static double convertDriveRot2M = (.0508) * 2 * Math.PI / 7.31;

        public static double wheelRadius = 0.028575;

        //Offset for each module in rotations
        public static double frAngleOffset = .076;
        public static double flAngleOffset = .102;
        public static double brAngleOffset = .172;
        public static double blAngleOffset = .591;

        // Formula is (-y,  x), where (x, y) is the untransformed coordinates
        // Ex: Front-right wheel original: (1, 3), Transformed: (-3, 1)
        public static Translation2d frRadius = new Translation2d(-0.3998, 0.3998);
        public static Translation2d flRadius = new Translation2d(-0.3998, -0.3998);
        public static Translation2d brRadius = new Translation2d(0.3998, 0.3998);
        public static Translation2d blRadius = new Translation2d(0.3998, -0.3998);

        public static int turnMotorCurrentLimit = 10;
        public static int driveMotorCurrentLimit = 35;

        public static double turnIntegrationCap = .5;
        public static double driveIntegrationCap = .001;

        public static double[] drivePIDGains = {0.006,0.000,0.0,.29};
        public static double[] turnPIDGains = {.005, 0, 0.0, 0};

    }
    public static class Drive{
        public static double maxDriveSpeedMpS = 3.5;        
        public static double maxTurnSpeedRpS = 1.5 * Math.PI;

        // public static double maxJoystickAccelXYMpS2 = maxDriveSpeedMpS * 1.5;        
        // public static double maxJoystickAccelThetaRpS2 = maxTurnSpeedRpS * 1.5;

        public static double maxJoystickAccelXYMpS2 = 4;//Double.MAX_VALUE;        
        public static double maxJoystckAccelThetaRpS2 = Math.PI * 3/2;


        public static int frDriveCANID = 8;
        public static int flDriveCANID = 6;
        public static int brDriveCANID = 2;
        public static int blDriveCANID = 4;

        public static int frTurnCANID = 7;
        public static int flTurnCANID = 5;
        public static int brTurnCANID = 41;
        public static int blTurnCANID = 3;

        public static boolean frDriveInvert = false;
        public static boolean frlDriveInvert = false;
        public static boolean brDriveInvert = false;
        public static boolean brlDriveInvert = false;
        
        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            //(y, -x) I have no clue why this is not the same as the one in Module
            new Translation2d(0.3998, -0.3998),
            new Translation2d(0.3998, 0.3998),
            new Translation2d(-0.3998, -0.3998),
            new Translation2d(-0.3998, 0.3998)
        );

        public static  DCMotor driveGearbox = DCMotor.getNEO(1);
    }
    public static class Robot{
        public static double L = 24.075;
        public static double thetaEst = 45;
        public static double offsetDeg = 13.544;
        public static double alphaRad = Units.degreesToRadians(163.57);
    }
    
    public static class Auto{
        public static PIDConstants xyController = new PIDConstants(4.5,0,0.01);
        public static PIDConstants thetaController = new PIDConstants(3.9,0,0.03);
        public static double maxDriveSpeed = 5;
        public static double robotMassKg = 74.088;
        public static double robotMOI = 6.883;
        public static double wheelCOF = 1.2;
        public static RobotConfig ppConfig =
            new RobotConfig(
                robotMassKg,
                robotMOI,
                new ModuleConfig(
                    Module.wheelRadius,
                    maxDriveSpeed,
                    wheelCOF,
                    Drive.driveGearbox.withReduction(6),
                    Module.driveMotorCurrentLimit,
                    1),
                Module.frRadius, Module.flRadius, Module.brRadius, Module.blRadius);
    }
}
