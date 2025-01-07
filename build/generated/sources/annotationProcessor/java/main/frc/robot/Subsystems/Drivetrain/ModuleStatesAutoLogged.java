package frc.robot.Subsystems.Drivetrain;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleStatesAutoLogged extends IModuleIO.ModuleStates implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TargetSpeedMpS", targetSpeedMpS);
    table.put("TargetAngleDeg", targetAngleDeg);
    table.put("CurrentSpeedMpS", currentSpeedMpS);
    table.put("CurrentDistanceM", currentDistanceM);
    table.put("CurrentAngleDeg", currentAngleDeg);
    table.put("DriveMotorTemp", driveMotorTemp);
    table.put("TurnMotorTemp", turnMotorTemp);
    table.put("DriveMotorOutputCurrent", driveMotorOutputCurrent);
    table.put("TurnMotorOutputCurrent", turnMotorOutputCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    targetSpeedMpS = table.get("TargetSpeedMpS", targetSpeedMpS);
    targetAngleDeg = table.get("TargetAngleDeg", targetAngleDeg);
    currentSpeedMpS = table.get("CurrentSpeedMpS", currentSpeedMpS);
    currentDistanceM = table.get("CurrentDistanceM", currentDistanceM);
    currentAngleDeg = table.get("CurrentAngleDeg", currentAngleDeg);
    driveMotorTemp = table.get("DriveMotorTemp", driveMotorTemp);
    turnMotorTemp = table.get("TurnMotorTemp", turnMotorTemp);
    driveMotorOutputCurrent = table.get("DriveMotorOutputCurrent", driveMotorOutputCurrent);
    turnMotorOutputCurrent = table.get("TurnMotorOutputCurrent", turnMotorOutputCurrent);
  }

  public ModuleStatesAutoLogged clone() {
    ModuleStatesAutoLogged copy = new ModuleStatesAutoLogged();
    copy.targetSpeedMpS = this.targetSpeedMpS;
    copy.targetAngleDeg = this.targetAngleDeg;
    copy.currentSpeedMpS = this.currentSpeedMpS;
    copy.currentDistanceM = this.currentDistanceM;
    copy.currentAngleDeg = this.currentAngleDeg;
    copy.driveMotorTemp = this.driveMotorTemp;
    copy.turnMotorTemp = this.turnMotorTemp;
    copy.driveMotorOutputCurrent = this.driveMotorOutputCurrent;
    copy.turnMotorOutputCurrent = this.turnMotorOutputCurrent;
    return copy;
  }
}
