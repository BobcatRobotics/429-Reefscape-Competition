// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final class Controllers {
    public static final int operator_controller_port = 2;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 9;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.2;
    public static final double ARM_SPEED_UP = -0.25;
    // public static final double ARM_HOLD_DOWN = 0.1;
    // public static final double ARM_HOLD_UP = -0.15;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 10;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_SLOW_SPEED_OUT = 0.1;
    public static final double ROLLER_FAST_SPEED_OUT = 0.2;
    public static final double ROLLER_SPEED_IN = -0.1;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 1;
    public static final double CLIMBER_SUPPLY = 50;
    public static final double CLIMBER_STATOR = 50;
    public static final double CLIMBER_SPEED_OUT = -0.6;
    public static final double CLIMBER_SPEED_IN = 0.6;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
