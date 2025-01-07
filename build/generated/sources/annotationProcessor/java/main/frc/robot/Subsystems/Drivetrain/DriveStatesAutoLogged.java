package frc.robot.Subsystems.Drivetrain;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveStatesAutoLogged extends IDriveIO.DriveStates implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GyroAngleDeg", gyroAngleDeg);
    table.put("OdomPoses", odomPoses);
    table.put("ChassisSpeeds", chassisSpeeds);
    table.put("TargetSpeeds", targetSpeeds);
    table.put("TargetAutoSpeeds", targetAutoSpeeds);
    table.put("RecievedNewControls", recievedNewControls);
  }

  @Override
  public void fromLog(LogTable table) {
    gyroAngleDeg = table.get("GyroAngleDeg", gyroAngleDeg);
    odomPoses = table.get("OdomPoses", odomPoses);
    chassisSpeeds = table.get("ChassisSpeeds", chassisSpeeds);
    targetSpeeds = table.get("TargetSpeeds", targetSpeeds);
    targetAutoSpeeds = table.get("TargetAutoSpeeds", targetAutoSpeeds);
    recievedNewControls = table.get("RecievedNewControls", recievedNewControls);
  }

  public DriveStatesAutoLogged clone() {
    DriveStatesAutoLogged copy = new DriveStatesAutoLogged();
    copy.gyroAngleDeg = this.gyroAngleDeg;
    copy.odomPoses = this.odomPoses.clone();
    copy.chassisSpeeds = this.chassisSpeeds.clone();
    copy.targetSpeeds = this.targetSpeeds.clone();
    copy.targetAutoSpeeds = this.targetAutoSpeeds.clone();
    copy.recievedNewControls = this.recievedNewControls;
    return copy;
  }
}
