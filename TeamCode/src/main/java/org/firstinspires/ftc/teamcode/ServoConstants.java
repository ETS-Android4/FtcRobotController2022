package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoConstants {
    // drive train servos
    public static double frMec = 0.10;
    public static double flMec = 0.992;
    public static double brMec = 0.95;
    public static double blMec = 0.0069;

    public static double frTank = 1;
    public static double flTank = 0;
    public static double brTank = 0;
    public static double blTank = 1;

    // intake servo limits
    public static double intakeMecDown = 0.095;
    public static double intakeTankDown = 0.140;
    public static double intakeUp = 0.57;

    // outtake heights
    // meccanumm heights
    public static int outtakeFirstLevelPosition = 0;
    public static int outtakeSecondLevelPosition = -120;
    public static int outtakeThirdLevelPosition = -345;

    // outtake servo limits
    public static double outtakeServoLowerLimit = 0.24;
    public static double outtakeServoLimitUpperMidHub = 0.9;
    public static double outtakeServoLimitLowerHub = 0.7;

    public static double outtakePower = 0.5;

    // capper servo limits
    public static double capStep = 0.05;
    public static double capStepSlow = 0.01;
    public static double capInit = 0;
    public static double capUp = 0;
    public static double capDown = 0.98;

}
