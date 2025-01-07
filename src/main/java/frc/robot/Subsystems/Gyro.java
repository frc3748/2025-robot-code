package frc.robot.Subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    // double offset = 0;
    AHRS navxAHRS = new AHRS(NavXComType.kUSB1, (byte) 50);
    LinearFilter filter = LinearFilter.movingAverage(2);
    public Gyro(int CANID){
        zero();
        navxAHRS.resetDisplacement();
    }
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(
            MathUtil.inputModulus(filter.calculate(-navxAHRS.getYaw()), 0, 360)
            );
    }

    public void testNavX(){
        SmartDashboard.putNumber("NAVx Test Angle", getAngle().getDegrees());
    }
    public void zero(){
        navxAHRS.zeroYaw();
        System.out.println("YAW Zeroed!!!!!!!");
    }
}
