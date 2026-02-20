package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    
    // Kraken motor controlling the turret
    private final TalonFX turretMotor;

    // PID controller for the angle of turret
    private final PIDController anglePidController;

    private final boolean absEncoderReversed;

    public final Commands commands = new Commands();

    public Turret(int turretID, int encoderID, boolean absEncoderReversed) {
        // Initializations
        this.turretMotor = new TalonFX(turretID);

        Preferences.initDouble("TurretP", TurretConstants.TURRETP);
        Preferences.initDouble("TurretD", TurretConstants.TURRETD);

        this.anglePidController = new PIDController(
            Preferences.getDouble("TurretP", TurretConstants.TURRETP),
            0,
            Preferences.getDouble("TurretD", TurretConstants.TURRETD)
        );

        this.anglePidController.setSetpoint(0);

        //this.absEncoder = new CANcoder(encoderID);
        this.absEncoderReversed = absEncoderReversed;

        Preferences.initDouble("TurretP", TurretConstants.TURRETP);
    }

    public double getAngle() {
        double rotations = this.turretMotor.getPosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absEncoderReversed) {
            angle.unaryMinus();
        }

        return angle.getRadians() * 11/136;
    }
    
    /*
     * Does some basic math to reset the turret's zero position
     *
    public void resetEncoder() {
        double offset = Rotation2d.fromRadians(Preferences.getDouble("TurretZero", 0)).plus(Rotation2d.fromRadians(getAngle())).getRotations();
        Preferences.setDouble("TurretZero", Rotation2d.fromRotations(offset).getRadians());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absEncoder.getConfigurator().apply(config);
    }*/

    @Override
    public void periodic() {
        // Gets the pid controller's output
        double currentAngle = this.getAngle();
        double calculatedOutput = this.anglePidController.calculate(currentAngle);

        // Makes sure the turret doesn't over extend
        boolean overrun = (
            (calculatedOutput < 0 && currentAngle < TurretConstants.TURRETMINANGLE) ||
            (calculatedOutput > 0 && currentAngle > TurretConstants.TURRETMAXANGLE)
        );

        // Sets the correct voltage for the motor
        double finalOutput = overrun ? 0 : calculatedOutput;
        this.turretMotor.setVoltage(finalOutput);
    }

    /*
     * Adds values that can be displayed on Elastic dashboard
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Turret Angle",
            () -> Units.radiansToDegrees(getAngle()),
            (double angle) -> this.anglePidController.setSetpoint(Units.degreesToRadians(angle))
        );
        builder.addDoubleArrayProperty(
            "Turret PID", 
            () -> new double[]{this.anglePidController.getP(), this.anglePidController.getD()}, 
            (double[] pd) -> {
                this.anglePidController.setP(pd[0]);
                this.anglePidController.setD(pd[1]);
            }
        );
    }

    public class Commands {
        public Command changeAngle(DoubleSupplier additionSupplier) {
            return Turret.this.run(
                () -> Turret.this.anglePidController.setSetpoint(Turret.this.anglePidController.getSetpoint() + Units.degreesToRadians(additionSupplier.getAsDouble()))
            ).finallyDo(
                () -> {}
            );
        }
    }
}
