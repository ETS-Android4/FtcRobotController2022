package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Constants.*;

@Config
public class Outtake {
    public DcMotorEx outtake;
    public Servo outtakeServo;

    // outtake constants
    double outtakeDelay = 0.2;
    public static int outtakeThirdLevelPosition = -360;
    public static int outtakeFirstLevelPosition = -120;
    public static int outtakeDownPosition = 0;
    public static double outtakePower = 0.5;

    public Outtake(HardwareMap hardwareMap) {
        // outtake servo init
        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
        outtakeServo.setDirection(Servo.Direction.FORWARD);

        //outtake motor init
        outtake = hardwareMap.get(DcMotorEx.class, "intake");

        //(DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
    }

    public void outtakeInit() {
        // initialize positions
        outtake.setTargetPosition(outtakeDownPosition);
        outtakeServo.setPosition(outtakeServoLowerLimit);
    }

    public void outtakeRaise (int level) {
        if (level == 1) {
            outtake.setTargetPosition(outtakeThirdLevelPosition);
        } else if (level == 3) {
            outtake.setTargetPosition(outtakeThirdLevelPosition);
        }

        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setPower(outtakePower);
    }

    public void outtakeDown() {
        outtake.setTargetPosition(outtakeDownPosition);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setPower(outtakePower*0.6);

        // servo ready for transfer
        outtakeServo.setPosition(outtakeServoLowerLimit);
    }

    public void outtakeRelease(int level) {
        if (level == 1) {
            outtakeServo.setPosition(outtakeServoLimitLowerHub);
        } else if (level == 2 || level == 3) {
            outtakeServo.setPosition(outtakeServoLimitUpperMidHub);
        }

    }

}
