package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limit Test")
@Config
public class LimitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setDirection(DcMotor.Direction.REVERSE);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo bl = hardwareMap.get(Servo.class, "outtake servo"); // limits: 0.015 and 1
        //intakePosition = hardwareMap.get(Servo.class, "intakeLift");
        DcMotorEx intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeExtension.setDirection(DcMotor.Direction.REVERSE);

        intakeExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servo s = hardwareMap.get(Servo.class, "stick");
        //Servo s = hardwareMap.get(Servo.class, "stopper");

//        Servo fl = hardwareMap.get(Servo.class, "frontleft");
//
//        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "left");
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setDirection(DcMotor.Direction.FORWARD);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            //bl.setPosition(servoPos);
//            intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            intakeExtension.setTargetPosition(100);
//            intakeExtension.setPower(0.5);
            telemetry.addData("pos: ", intakeExtension.getCurrentPosition());
//            outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtake.setTargetPosition(-120);
//            outtake.setPower(0.5);

            //s.setPosition(servoPower);
            //fl.setPosition(0.0);

            //motor2.setVelocity(1000);
//            motor.setPower(0.1);
//
//            if (motor.getCurrentPosition() == -330) {
//                motor.setPower(0.0);
//            }

            //telemetry.addData("position",motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
