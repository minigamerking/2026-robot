// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVERCONTROLLERPORT = 0;

    public static final double DEADBAND = 0.05;
  }

  public static class ShooterConstants {
    public static final int SHOOTERMOTORONEID = 1;
    public static final int SHOOTERMOTORTWOID = 2;
  }
  
  public static class TurretConstants {
    public static final int TURRETP = 1;
    public static final int TURRETI = 0;
    public static final int TURRETD = 0;

    public static final double TURRETMAXANGLE = 90;
    public static final double TURRETMINANGLE = 0;
  }

  public static class SwerveConstants {
    public static final double WHEELDIAMETERMETERS = Units.inchesToMeters(4);

    public static final int SWERVETURNINGP = 1;
    public static final int SWERVETURNINGI = 0;
    public static final int SWERVETURNINGD = 0;

    public static final double PHYSICALMAXSPEEDMPERSECR1 = 14.4;
    public static final double PHYSICALMAXSPEEDMPERSECR2 = 16.8;
    public static final double PHYSICALMAXSPEEDMPERSECR3 = 19.2;

    public static final double DRIVEMOTORGEARRATIOR1 = 7.03;
    public static final double DRIVEMOTORGEARRATIOR2 = 6.03;
    public static final double DRIVEMOTORGEARRATIOR3 = 5.27;
    public static final double STEERINGGEARRATIO = 26;

    public static final double ROTATIONSTOMETERSR1 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR1;
    public static final double ROTATIONSTOMETERSR2 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR2;
    public static final double ROTATIONSTOMETERSR3 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR3;
  }
}
