package frc.robot.Utilities;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Subsystems.Drivetrain.IDriveIO.DriveStates;

public class NonlinearArmSolver implements Sendable {
    double _a;
    double _b;

    static LinearFilter speedFilter = LinearFilter.movingAverage(3);
    private static double lastAvgSpeed = 17.5;

    // static double[] robotToPivot = {-.229,0.193};
    static double pivotOffset = -0.229;
    static double[] getRTheta(double xField, double yField, double thetaGuess){
        double deltaX = xField + Constants.Robot.L*Math.cos(thetaGuess);
        double deltaY = yField - Constants.Robot.L*Math.sin(thetaGuess);
        double[] rTheta = {Math.sqrt(deltaX * deltaX + deltaY * deltaY), Math.atan(Math.abs(deltaY / deltaX))};

        return rTheta;
    }

    static double getAngleShooterToHorizontal(double r){
        r = Units.inchesToMeters(r);
        // // Assume 20m/s
        double b = .00548;
        double a = -0.0159;

        // Assume 16.5m/s
        // double b = 0.00493;
        // double a = -0.0221;
        //Assume 8m/s
        // double a = -0.0813;
        // double b = 0.0033;

        // double b = 0.00066 + 0.000438 * lastAvgSpeed - 0.00000987 * lastAvgSpeed * lastAvgSpeed;
        // double a = -0.256 + 0.0345*lastAvgSpeed - 0.00174 * lastAvgSpeed * lastAvgSpeed + 0.0000308 * lastAvgSpeed * lastAvgSpeed * lastAvgSpeed;
    
        return Math.abs(Math.atan(a*r + b));
    }

    static double distance(double xField, double yField, double guess){
        double[] rTheta = NonlinearArmSolver.getRTheta(xField, yField, guess);

        return (Constants.Robot.alphaRad - (getAngleShooterToHorizontal(rTheta[0]) + rTheta[1]) - Math.PI / 2 - Units.degreesToRadians(Constants.Robot.offsetDeg) - guess);
    }

    static double DerDistance(double guess, double xField, double yField){
        return (distance(xField, yField, guess + 0.0001) - distance(xField, yField, guess)) / 0.0001;
    }

    public static double solve(double xField, double yField){
        double guess = Math.PI / 4;

        for(int i = 0; i < 3; i ++){
            guess = -distance(xField, yField, guess) / DerDistance(guess, xField, yField) + guess;
        }

        return MathUtil.clamp(Units.radiansToDegrees(guess), 0, 80);
    }
    public static double solveWithVision(Supplier<Pose2d> poseSupplier){
        double[] pos = getPivotPosition(poseSupplier);
        return solve(pos[0], pos[1]);
    }
    public static double[] getPivotPosition(Supplier<Pose2d> robotPoseSupplier){
        Pose2d _pose = robotPoseSupplier.get();
        double[] speakerPose;
        double[] robotPose = {_pose.getX(), _pose.getY()}; 
        try{
            speakerPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 
                new double[]{16.579342 - Units.inchesToMeters(9.3), 5.547867999999999}: 
                new double[]{-0.03809999999999 + Units.inchesToMeters(9.5), 5.547867999999999};
        }catch(Exception e){
            DriverStation.reportWarning(e.getMessage(), false);
            speakerPose = new double[]{16.579342, 5.547867999999999};
        }

        double distance = Math.sqrt(Math.pow(robotPose[0] - speakerPose[0], 2) + Math.pow(robotPose[1] - speakerPose[1], 2));
        
        return new double[]{Units.metersToInches(distance + pivotOffset), 69.5}; // Added 6 inch offset *Might change with recalibration
    }
    private void updateShotSpeed(double lastShotSpeed){
        lastAvgSpeed = speedFilter.calculate(lastShotSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("X-dist to pivot", () -> _a, (double _aNew)-> {_a = _aNew;});
        builder.addDoubleProperty("Y-dist to pivot", () -> _b, (double _bNew) -> {_b = _bNew;});
        builder.addDoubleProperty("Estimated Angle", () -> NonlinearArmSolver.solve(_a, _b), null);
        builder.addBooleanProperty("Set Angle", () -> false, (boolean on) -> {
            if(on){     
                SmartDashboard.putNumber("ArmSetpoint", solve(_a, _b));
            }
        });
    }
}
