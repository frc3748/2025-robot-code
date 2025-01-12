package frc.robot.Commands;

import java.lang.annotation.Target;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.LimeLightHelpers;

public class AimAtTag extends  Command{
    ProfiledPIDController pid;
    Drive drive;
    double target;
    double current;
    boolean flag = false;
    

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
 
    //post to smart dashboard periodically

    public AimAtTag(Drive drive){
        pid = new ProfiledPIDController(0.3, 0, 0, new Constraints(Math.PI / 2, Math.PI));
       
        this.drive = drive;
    }

    public void initialize(){
        //targetAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians() + this.getTargetSpeakerAngDouble(), 0, Math.PI * 2);
        target = getTagAngle();
        //current = MathUtil.inputModulus(drive.getGyroAngle().getRadians(), 0, Math.PI * 2);
        current = drive.getPose().getX();
        pid.reset(current); //(currentAngle);
    }

    // sets robotSpeed in radians per second
    public void execute(){
        SmartDashboard.putNumber("Current Angle", 0.5);
        System.out.println(pid.calculate(current,target));
        // currentAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians(), 0, 2 * Math.PI);
        // SmartDashboard.putNumber("Target angle", Math.toDegrees(targetAngle));
        // SmartDashboard.putNumber("Current angle", Math.toDegrees(currentAngle));
        drive.setRobotSpeeds(new ChassisSpeeds(pid.calculate(current, target), 0, 0));
    }

    // get angle to aim at shooter
    public double getTagAngle(){
        return LimeLightHelpers.getTX("");
    }
    
        
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


