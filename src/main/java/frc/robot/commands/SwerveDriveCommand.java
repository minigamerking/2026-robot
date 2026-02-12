package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    private final SwerveSubsystem swerve;


    public SwerveDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, SwerveSubsystem swerve) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.swerve = swerve;
        this.addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double rotation = rotSupplier.getAsDouble();

        x = Math.abs(x) > OperatorConstants.DEADBAND ? x : 0.0;
        y = Math.abs(y) > OperatorConstants.DEADBAND ? y : 0.0;
        rotation = Math.abs(rotation) > OperatorConstants.DEADBAND ? rotation : 0.0;

        x *= swerve.maxSpeed * SwerveConstants.TELEOPSPEEDMULTIPLIER;
        y *= swerve.maxSpeed * SwerveConstants.TELEOPSPEEDMULTIPLIER;
        rotation *= swerve.maxSpeed  * SwerveConstants.TELEOPSPEEDMULTIPLIER;

        //System.out.println("X: " + x + " Y: " + y + " Rotation: " + rotation);

        swerve.drive(x, y, rotation);
    }
}
