package frc.robot.Subsystems.Drivetrain;


import org.littletonrobotics.junction.AutoLog;


public interface IModuleIO {
    
    @AutoLog
    public static class ModuleStates{
        double targetSpeedMpS;
        double targetAngleDeg;
        double currentSpeedMpS;
        double currentDistanceM;
        double currentAngleDeg;

        double driveMotorTemp;
        double turnMotorTemp;

        double driveMotorOutputCurrent;
        double turnMotorOutputCurrent;
    }

    public ModuleStatesAutoLogged updateStates(ModuleStatesAutoLogged _states);
    //public default void updateStates(ModuleStates _states) {}
}
