package frc.robot.Commands.AutoSpecific;

import java.util.Set;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shintake.Shintake;

public class ContinuosShoot extends Command {
    Shintake shintake;
    double targetVelocity;
    double tolerance = 1;

    public enum GlobalState{
        SPIN_UP,
        IDLE,
        INTAKE,
        SHOOT,
    }
    enum IntakeState{
        WAITING,
        CENTERING,
        HOLDING
    }

    public GlobalState state = GlobalState.SPIN_UP;
    IntakeState intakeState = IntakeState.WAITING;

    Timer intakeTimer = new Timer();

    public ContinuosShoot(Shintake _shintake, double _targetSpeed){
        this.shintake = _shintake;
        this.targetVelocity = _targetSpeed;
    }

    @Override
    public void initialize(){
        shintake.stopIntake();
        shintake.stopShooter();
        shintake.resetIntakeEncoder();
        shintake.resetShooterEndocer();

        shintake.setShooterIdleMode(IdleMode.kCoast);

        shintake.setShooterVelocity(targetVelocity);
        state = GlobalState.SPIN_UP;
        intakeState = IntakeState.WAITING;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("GlobalState", state.toString());
        SmartDashboard.putString("IntakeState", state.toString());
        if((Math.abs(targetVelocity - getAvgShooterVelocities()) < tolerance) && state == GlobalState.SPIN_UP){
            state = GlobalState.IDLE;
        } if(shintake.getIntakePosition() < -2.25 && state == GlobalState.SHOOT){
            state = GlobalState.SPIN_UP;
        } 
        boolean finish = (intakeState == IntakeState.HOLDING && shintake.getIntakePosition() >= Preferences.getDouble("Shintake/intakeHoldPoint", 0.075)) || intakeTimer.get() > 10;
        if(state == GlobalState.INTAKE && finish){
            shintake.stopIntake();
            shintake.stopShooter();
            intakeState = IntakeState.WAITING;
            state = GlobalState.SPIN_UP;
        }

        switch (state) {
            case SPIN_UP:
                SPIN_UP();
                break;
            case IDLE:
                IDLE();
                break;
            case INTAKE:
                INTAKE();
                break;
            case SHOOT:
                SHOOT();
                break;
        }
    }

    public double getAvgShooterVelocities(){
        double[] shooterVelocities = shintake.getShooterVelocities();
        return (shooterVelocities[0] + shooterVelocities[1]) / 2;
    }
    public ContinuosShoot setShooterVelocityTolerance(double tolerance){ this.tolerance = tolerance; return this; }
    private void SPIN_UP(){
        IDLE();
    }
    private void IDLE(){
        shintake.setShooterVelocity(targetVelocity);
        // shintake.stopShooter();
        shintake.stopIntake();
        shintake.resetIntakeEncoder();
    }
    private void INTAKE(){
        if (shintake.getTOFDistance() < 90 && intakeState == IntakeState.WAITING){
            intakeState = IntakeState.CENTERING;
            shintake.stopIntake();
        }
        if(intakeState == IntakeState.CENTERING && this.shintake.getIntakePosition() <= -.1){
            intakeState = IntakeState.HOLDING;
        }


        switch (intakeState) {
            case WAITING:
                shintake.setIntakeSpeed(Preferences.getDouble("Shintake/intakeSpeed", -3));
                shintake.resetIntakeEncoder();
                break;
            case CENTERING:
                shintake.setIntakeSpeed(-1);
                RobotContainer.armHandler.updateSetpoint(13);
                break;
            case HOLDING:
                shintake.setIntakeSpeed(.75);
                RobotContainer.armHandler.updateSetpoint(13);
                break;
        }

        shintake.stopShooter();
    }
    private void SHOOT(){
        if((Math.abs(targetVelocity - getAvgShooterVelocities()) < tolerance)){
            shintake.setShooterVelocity(targetVelocity);
            shintake.setIntakeSpeed(-5);
        }else{
            IDLE();
        }
    }

    public ContinuosShoot setState(GlobalState _state){
        // if(state == GlobalState.SPIN_UP){ return this; }
        switch (_state) {
            case INTAKE:
                intakeState = IntakeState.WAITING;
                intakeTimer.restart();
                shintake.setIntakeSpeed(Preferences.getDouble("Shintake/intakeSpeed", -3));
                break;
            default:
                break;
        }
        state = _state;
        return this;
    }
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(shintake);
    }

    @Override
    public void end(boolean interrupted) {
        shintake.stopIntake();
        shintake.stopShooter();
    }
}
