package frc.robot.Commands;

import java.lang.annotation.Target;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.LimeLightHelpers;
import frc.robot.Utilities.LimeLightHelpers.RawFiducial;

public class AimAtTag extends  Command{
    PIDController pid;

    Drive drive;
    double target;
    double current;
    boolean flag = false;

   //post to smart dashboard periodically

    public AimAtTag(Drive drive){
        pid = new PIDController(0.00005, 0.05, 0.0001);
        pid.enableContinuousInput(0, 360);




        this.drive = drive;
    }

    public void initialize(){
        current =  drive.getGyroAngle().getDegrees();
        //targetAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians() + this.getTargetSpeakerAngDouble(), 0, Math.PI * 2);
        pid.reset(); //(currentAngle);
    }

    // sets robotSpeed in radians per second
    public void execute(){
        current =  drive.getGyroAngle().getDegrees();
        SmartDashboard.putNumber("Current Angle", 0.5);
        System.out.println(current);

        LimeLightHelpers.RawFiducial[] aprilTagTargetResults = LimeLightHelpers.getRawFiducials("");
        boolean hasTag = false;
    
        if(LimeLightHelpers.getTV("")){
            for (LimeLightHelpers.RawFiducial f: aprilTagTargetResults){
                if(f.id == 7){
                    target = 60;
                    hasTag = true;
                }
            }
        }
        if(!hasTag){
            //System.out.println("Unable to find a a target");
            target = 60;
            //System.out.println(pid.calculate(current, target));
            // drive.setRobotSpeeds(new ChassisSpeeds(0, 0, pid.calculate(current, target)));           
            // return;
        }
        
        System.out.println("PID Estimate: " + pid.calculate(current, target));
        System.out.println("Current Angle: " + current);

        drive.setRobotSpeeds(new ChassisSpeeds(0, 0, pid.calculate(current, target)));
        // drive.setRobotSpeeds(new ChassisSpeeds(0, 0, pid.calculate(current, target)));

        
        // SmartDashboard.putNumber("Pid Estimate", pid.calculate(current, target));
        // SmartDashboard.putNumber("Target angle", Math.toDegrees(target));
        // SmartDashboard.putNumber("Current angle", Math.toDegrees(current));
       //drive.setRobotSpeeds(new ChassisSpeeds(0, pid.calculate(current, target), 0));
    }

    // get angle to aim at shooter
    
    
        
        // Documentation on Alliance station (https://docs.wpilib.org/en/stable/docs/software/basic-programming/alliancecolor.html)
        // this basically stores the position where speakers were at the battle of baltimore
        // for the coral reef one, I think we should set a central point for both team Red and team Blue
        // afterwards, we can manually put offsets for each reef holder, offset for the ball
        // we can copy the same logic for the new "speakers" where we will put the balls in, just change the position
       

        // Transform2d robotToSpeaker = speakerPos.minus(integratedRobotPose);
        // angle = Math.atan2(robotToSpeaker.getY(), robotToSpeaker.getX());
      

        

    @Override
    public Set<Subsystem> getRequirements(){
        return Set.of(drive);
    }
}


