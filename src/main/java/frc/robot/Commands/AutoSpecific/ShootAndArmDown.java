package frc.robot.Commands.AutoSpecific;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.ShooterAuto;
import frc.robot.Subsystems.Shintake.Shintake;

public class ShootAndArmDown extends ShooterAuto {

    public ShootAndArmDown(Shintake _shintake, Supplier<Pose2d> poseSupplier) {
        super(_shintake, poseSupplier);
    }

    public static SequentialCommandGroup getCommand(Shintake _shintake, Supplier<Pose2d> poSupplier){
        double armSetpoint = Constants.Arm.dynamicPersistant.getDouble("IntakeArmPosition");
        return new SequentialCommandGroup(new ShootAndArmDown(_shintake, poSupplier),
        new InstantCommand(() -> RobotContainer.armHandler.updateSetpoint(armSetpoint)).repeatedly().until(
            () -> Math.abs(armSetpoint - RobotContainer.armHandler.getPosition().getDegrees()) > 0.75
        ));
    }

    

}