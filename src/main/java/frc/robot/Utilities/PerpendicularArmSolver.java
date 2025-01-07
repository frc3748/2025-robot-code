package frc.robot.Utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class PerpendicularArmSolver implements Sendable {
    private static double _a;
    private static double _b;

    private static double equation(double thetaEst, double a, double b){
        return -thetaEst + 163.57 - Math.atan(
            Math.abs(
                (b-Constants.Robot.L*Math.sin(thetaEst * Math.PI / 180))/(a+Constants.Robot.L*Math.cos(thetaEst * Math.PI / 180))
            )
        ) * 180 / Math.PI - 90;
    }

    private static double DerEquation(double theta, double a, double b, double dt){
        return (equation(theta + dt, a, b) - equation(theta, a, b)) / dt;
    }

    public static double solve(double a, double b){
        double guess = Constants.Robot.thetaEst;
        for(int i = 0; i < 3; i++){
            guess = -equation(guess, a, b) / DerEquation(guess, a, b, 0.001) + guess;
        }

        return guess - Constants.Robot.offsetDeg;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("X-dist to pivot", () -> _a, (double _aNew)-> {_a = _aNew;});
        builder.addDoubleProperty("Y-dist to pivot", () -> _b, (double _bNew) -> {_b = _bNew;});
        builder.addDoubleProperty("Estimated Angle", () -> solve(_a, _b), null);
        builder.addBooleanProperty("Set Angle", () -> false, (boolean on) -> {
            if(on){     
                SmartDashboard.putNumber("ArmSetpoint", solve(_a, _b));
            }
        });
    }
}
