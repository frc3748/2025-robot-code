package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Drivetrain.Drive;

public class AimAtSpeaker extends Command{
    
    ProfiledPIDController pid;
    Drive drive;
    double targetAngle;
    double currentAngle;
    boolean flag = false;

    public AimAtSpeaker(Drive drive){
        pid = new ProfiledPIDController(0.3, 0, 0, new Constraints(Math.PI / 2, Math.PI));
        pid.setTolerance(Math.toRadians(2));
        pid.enableContinuousInput(0, 2*Math.PI);
        this.drive = drive;
    }

    public void initialize(){
        targetAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians() + this.getTargetSpeakerAngDouble(), 0, Math.PI * 2);
        currentAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians(), 0, Math.PI * 2);
        pid.reset(currentAngle); //(currentAngle);
    }

    // sets robotSpeed in radians per second
    public void execute(){
        SmartDashboard.putNumber("Current Angle", 0.5);
        currentAngle = MathUtil.inputModulus(drive.getGyroAngle().getRadians(), 0, 2 * Math.PI);
        SmartDashboard.putNumber("Target angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Current angle", Math.toDegrees(currentAngle));
        drive.setRobotSpeeds(new ChassisSpeeds(0, 0, pid.calculate(currentAngle, targetAngle)));
    }

    // get angle to aim at shooter
    public Double getTargetSpeakerAngDouble(){
        double angle;
        Pose2d speakerPos;
        
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                speakerPos = new Pose2d(16.579342, 5.547868, new Rotation2d());
            } else {
                speakerPos = new Pose2d(-0.038099, 5.547868, new Rotation2d());
            }
        } else {
            return null;
        }

        // Transform2d robotToSpeaker = speakerPos.minus(integratedRobotPose);
        // angle = Math.atan2(robotToSpeaker.getY(), robotToSpeaker.getX());
        double robotX = Math.cos(drive.getPose().getRotation().getRadians());        
        double robotY = Math.sin(drive.getPose().getRotation().getRadians());
        double robotToSpeakerX = speakerPos.getX() - drive.getPose().getX();        
        double robotToSpeakerY = speakerPos.getY() - drive.getPose().getY();

        angle = Math.asin((robotToSpeakerX * robotY - robotToSpeakerY * robotX) / Math.hypot(robotX, robotY) /Math.hypot(robotToSpeakerX, robotToSpeakerY)); 
        if (Double.isFinite(angle)){
            // if (angle < 0) angle += Math.PI;
            //angle += Math.PI;
            return Double.valueOf(angle + Math.toRadians(180));
        }
        return null;
    }

    public boolean isFinished(){
        if (Math.abs(targetAngle - currentAngle) < Math.toRadians(3)){
            SmartDashboard.putNumber("Target angle: ", targetAngle);
            SmartDashboard.putNumber("Current Angle: ", currentAngle);
            System.out.println("PID SETPOINT FINISHED" + String.valueOf(pid.atSetpoint()));
            return true;
        }
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements(){
        return Set.of(drive);
    }
}
