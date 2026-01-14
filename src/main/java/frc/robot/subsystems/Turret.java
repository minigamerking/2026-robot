package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Turret extends SubsystemBase {
    
    private final TalonFX turretMotor;

    private final PIDController anglePidController;

    private final CANcoder absEncoder;

    public Turret(int turretID, int encoderID) {
        this.turretMotor = new TalonFX(turretID);

        this.anglePidController = new PIDController(
            SubsystemConstants.turretP,
            SubsystemConstants.turretI,
            SubsystemConstants.turretD
        );

        this.anglePidController.setSetpoint(0);

        this.absEncoder = new CANcoder(encoderID);
    }
    

}
