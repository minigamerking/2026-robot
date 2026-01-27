package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class SwerveDriveCommand extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;


    public SwerveDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double rotation = rotSupplier.getAsDouble();

        x = Math.abs(x) > OperatorConstants.DEADBAND ? x : 0.0;
        y = Math.abs(y) > OperatorConstants.DEADBAND ? y : 0.0;
        rotation = Math.abs(rotation) > OperatorConstants.DEADBAND ? rotation : 0.0;

        //swerve.drive(x, y, rotation);
    }
}
