package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;


public interface IDriveIO {
    @AutoLog
    public class DriveStates{
        double gyroAngleDeg;
        double[] odomPoses = {0, 0, 0};

        double[] chassisSpeeds = new double[3];

        double[] targetSpeeds = {0,0,0};
        double[] targetAutoSpeeds = {0, 0, 0};

        boolean recievedNewControls;
    }
    public void updateDriveStates(DriveStatesAutoLogged states);
}
