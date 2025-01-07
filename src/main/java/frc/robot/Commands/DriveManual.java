package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;

public class DriveManual extends Command{
    Joystick joy;
    Drive driveSubsystem;
    SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);      
    SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);    
    SlewRateLimiter thetaLimiter = new SlewRateLimiter(Constants.Drive.maxJoystckAccelThetaRpS2);


    public DriveManual(Drive driveSubsystem, Joystick joy){
        this.driveSubsystem = driveSubsystem;
        this.joy = joy;
    }
    public ChassisSpeeds getChassisSpeeds(){

        double joyX = -joy.getRawAxis(1);
        double joyY = -joy.getRawAxis(0);
        double joyZ = joy.getRawAxis(2);

        joyX = MathUtil.applyDeadband(joyX, .05);
        joyY = MathUtil.applyDeadband(joyY, .05);
        joyZ = MathUtil.applyDeadband(joyZ, .05);

        joyX = xLimiter.calculate(joyX * joyX * Math.signum(joyX));
        joyY = yLimiter.calculate(joyY * joyY * Math.signum(joyY));
        joyZ = thetaLimiter.calculate(joyZ * joyZ * Math.signum(joyZ));

        return new ChassisSpeeds(joyX * Constants.Drive.maxDriveSpeedMpS , joyY * Constants.Drive.maxDriveSpeedMpS , joyZ * Constants.Drive.maxTurnSpeedRpS );
      }

    @Override
    public void execute() {
          double multiplier = joy.getRawButton(6) ? .375: 1;
        if(joy.getRawButton(5)){
          driveSubsystem.setRobotSpeeds(getChassisSpeeds().times(multiplier));
        }else{
          driveSubsystem.SetFieldSpeeds(getChassisSpeeds().times(multiplier));
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(driveSubsystem);
    }
}
