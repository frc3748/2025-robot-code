package frc.robot.Commands.AutoSpecific;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeAuto;
import frc.robot.Subsystems.Shintake.Shintake;

public class PositionedIntake extends IntakeAuto{
    Shintake shintake;
    public PositionedIntake(Shintake shintake){
        super(shintake);
        this.shintake = shintake;
    }

    // Put arm on the floor
    public static SequentialCommandGroup getSequentialCommand(Shintake shintake) {
        double intakeSetpoint = Constants.Arm.dynamicPersistant.getDouble("IntakeArmPosition");
        return new SequentialCommandGroup(new InstantCommand(
            () -> RobotContainer.armHandler.updateSetpoint(intakeSetpoint), shintake
        ).repeatedly().until(() -> Math.abs(RobotContainer.armHandler.getPosition().getDegrees() - intakeSetpoint) < .75),  
        new PositionedIntake(shintake));
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(shintake);
    }
}
