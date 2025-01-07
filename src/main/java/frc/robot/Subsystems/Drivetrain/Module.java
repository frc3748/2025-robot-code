package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Module implements IModuleIO, Subsystem, Sendable {
    private static int numModTracker = 0;
    
    SparkMax driveMotor;
    SparkMaxConfig driveConfig;
    SparkClosedLoopController drivePID;
    RelativeEncoder driveEncoder;
    
    SparkMax turnMotor;
    SparkMaxConfig turnConfig;
    AbsoluteEncoder turnAbsEncoder;
    SparkClosedLoopController turnPID;

    double targetAngle;
    double targetSpeed;
    double turnError;

    double angleOffset;

    private final ModuleStatesAutoLogged states = new ModuleStatesAutoLogged();
    

    Translation2d radius;

    String moduleID;
    // Initialization Module Object
    public Module(
        int driveID,
        int turnID,
        Translation2d radius,
        double angleOffset,
        boolean flipDrive,
        String modID
    ){

        //Drive motor config
        driveMotor =  new SparkMax(driveID, MotorType.kBrushless);
        driveConfig = new SparkMaxConfig();
        drivePID = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0);

        driveConfig
            .inverted(flipDrive)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Module.driveMotorCurrentLimit);
        driveConfig.encoder
            .positionConversionFactor(Constants.Module.convertDriveRot2M)
            .velocityConversionFactor(Constants.Module.convertDriveRot2M / 60);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(Constants.Module.drivePIDGains[0],  Constants.Module.drivePIDGains[1], Constants.Module.drivePIDGains[2], Constants.Module.drivePIDGains[3])
            .iMaxAccum(Constants.Module.driveIntegrationCap);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //turn motor config
        turnMotor =  new SparkMax(turnID, MotorType.kBrushless);
        turnConfig = new SparkMaxConfig();
        turnAbsEncoder = turnMotor.getAbsoluteEncoder();
        turnPID = turnMotor.getClosedLoopController();

        turnConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Module.turnMotorCurrentLimit);
        turnConfig.absoluteEncoder
            .positionConversionFactor(360)
            .velocityConversionFactor(60);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(Constants.Module.turnPIDGains[0], Constants.Module.turnPIDGains[1], Constants.Module.turnPIDGains[2], Constants.Module.turnPIDGains[3])
            .positionWrappingEnabled(true)
            .positionWrappingMaxInput(360)
            .positionWrappingMinInput(0);
    
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnMotor.clearFaults();
        driveMotor.clearFaults();



        // Make sure Absolute encoder data is up to date
        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);        
        // turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        this.angleOffset = Units.rotationsToDegrees(angleOffset);

        this.radius = radius;

        this.moduleID = modID;
        Module.numModTracker += 1;

    }
    public Module(
        int driveID,
        int turnID,
        Translation2d radius,
        double angleOffset,
        boolean flipDrive
    ){
        this(driveID, turnID, radius, angleOffset, flipDrive, String.valueOf(Module.numModTracker));
    }
    @Override
    public void periodic(){
        updateStates(states);
        Logger.processInputs(moduleID, states);
    }
    @Override
    public ModuleStatesAutoLogged updateStates(ModuleStatesAutoLogged _states) {
        double newAngle = getAngle().getDegrees();
        double newDistance = driveEncoder.getPosition();
        _states.currentAngleDeg = Math.abs(newAngle - _states.currentAngleDeg) / .02 > 1000 ? _states.currentAngleDeg : newAngle;
        _states.currentSpeedMpS = driveEncoder.getVelocity();
        _states.currentDistanceM = Math.abs(newDistance - _states.currentDistanceM) / .02 > 10 ? _states.currentDistanceM : newDistance;


        _states.targetAngleDeg = this.targetAngle;
        _states.targetSpeedMpS = this.targetSpeed;

        _states.turnMotorTemp =  turnMotor.getMotorTemperature();
        _states.driveMotorTemp =  driveMotor.getMotorTemperature();

        _states.turnMotorOutputCurrent = turnMotor.getOutputCurrent();
        _states.driveMotorOutputCurrent = driveMotor.getOutputCurrent();
        return _states;
    }

    public void setAngle(double degrees){
        if(!Double.isFinite(degrees)){ 
            this.stopTurnMotor();
            return;
         }

        degrees %= 360; 
        this.targetAngle = degrees;
        degrees += angleOffset;
        turnPID.setReference(degrees, ControlType.kPosition);
    }

    // Sets the drive speed
    public void setSpeed(double speedMpS){
        if(!Double.isFinite(speedMpS)){
            stopDriveMotor();
        }
        this.targetSpeed = speedMpS;
        drivePID.setReference(speedMpS, ControlType.kVelocity);
    }

    /**
     * Inverts speed and matches degrees + 180 if module is closer to this point.
     * @param speedMpS Target module speed. Must be positive
     * @param degrees Target module degrees
     */
    public void setSpeedAndAngle(double speedMpS, double degrees){
        // degrees = MathUtil.inputModulus(degrees, 0, 360);
        // boolean negate = negateVelocity(speedMpS, degrees);
        // this.setSpeed(negate ? -1 * speedMpS: speedMpS);
        // this.setAngle(negate ? (degrees + 180) % 360 : degrees);
        if(!Double.isFinite(speedMpS) || !Double.isFinite(degrees)){
            stop();
            return;
        }

        degrees = MathUtil.inputModulus(degrees, 0, 360);
        double targetVelocity = speedMpS * Math.cos(Units.degreesToRadians(degrees) - getAngle().getRadians());

        this.setSpeed(targetVelocity);
        this.setAngle(targetVelocity < 0 ? (degrees + 180) % 360: degrees);
    }

    public boolean negateVelocity(double targetSpeed, double targetAnlge){
        double positionError = MathUtil.inputModulus(targetAnlge - getAngle().getDegrees(), -180, 180);
        if(Math.abs(positionError) >= 90){
            return true;
        }else {
            return false;
        }
    }

    public void stopTurnMotor(){
        this.turnMotor.stopMotor();
    }

    public void stopDriveMotor(){
        this.driveMotor.stopMotor();
    }
    
    public void stop(){
        this.stopDriveMotor();
        this.stopTurnMotor();
    }

    public double getDriveDistance(){ 
        return states.currentDistanceM; 
    }

    public double getDriveSpeed(){ 
        return states.currentSpeedMpS; 
    }

    public Rotation2d getAngle(){
        double degrees = MathUtil.inputModulus(turnAbsEncoder.getPosition() - angleOffset, 0, 360);
        return Rotation2d.fromDegrees(degrees);
    }

    public Rotation2d getTargetAngle(){ 
        return Rotation2d.fromDegrees(states.targetAngleDeg); 
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("GeneralSubsystem");
        builder.addDoubleProperty("Module Angle", () -> getAngle().getDegrees(), null);
        builder.addDoubleProperty("Module Distance", this::getDriveDistance, null);
        builder.addDoubleProperty("Module Speed", this::getDriveSpeed, null);
        builder.addDoubleProperty("CurrentTargetAngle", () -> this.getTargetAngle().getDegrees(), null);
    }

}

