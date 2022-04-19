package org.firstinspires.ftc.teamcode.auto.blue;

import static org.firstinspires.ftc.teamcode.ServoConstants.outtakeFirstLevelPosition;
import static org.firstinspires.ftc.teamcode.ServoConstants.outtakeSecondLevelPosition;
import static org.firstinspires.ftc.teamcode.ServoConstants.outtakeThirdLevelPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CVPipeline;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "BlueCringe")
public class BlueCringe extends LinearOpMode {
    OpenCvWebcam webcam;
    CVPipeline pipeline;

    DcMotorEx intakeExtension, carousel, fleft, fright, bleft, bright, outtake, intakeSurgical;

    public SampleMecanumDrive mecanumDrive;
    public SampleTankDrive tankDrive;
    double mecDown = 0.92;

    double tankDown = 0.85;
    double time = 0.0;
    double intakeUp = 0.15;
    int position = 1;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Pose2d startingPosition = new Pose2d(-58, 56, Math.toRadians(-67));

    Trajectory goToShippingHubFromCarousel2, goToTeammateItemFromShippingHub,
            goToShippingHubFromAlliance, goToCarouselFromStarting, goToShippingHubFromCarousel,
            goToAllianceFreightFromShippingHub, goToTeammateItemFromShippingHubPart1,
            goToTeammateItemFromShippingHubPart2, goToTeammateItemFromShippingHubPart3,
            goToTeammateItemFromShippingHubPart4, goToFreightFromShippingHub,
            goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos,
            goToAllianceFreightFromShippingHub2, shippingHubAll1, shippingHubAll2, park1, park2,
            start1, start2;

    Servo fr, br, fl, bl, outtakeServo, intakePosition;

    public static double outtakePower = 0.5;
    public static double outtakeServoClosePosition = 0.2;
    public static double outtakeServoOpenPosition = 0.7;
    public static int outtakeDownPosition = 0;

    public static boolean isMec = false;

    enum State {
        PICK_UP_FREIGHT,
        TURN_AT_FREIGHT,
        GO_TO_SWITCHING_POS,
        GO_TO_STORAGE_UNIT,
        OUTTAKE_FREIGHT,
        GO_TO_SWITCHING_POS_2,
        DRIVE_TO_FREIGHT_2,
        PARK_1, PARK_2,
        INTAKE_ALLIANCE,
        IDLE, GO_TO_SHIPPING_HUB_2, GO_TO_SHIPPING_HUB_TURN_2, TRANSITION_SCORE_FREIGHT_IN_SHIPPING_HUB,
        SHIPPING_HUB_ALL1, SHIPPING_HUB_ALL2,
        START_1, START_2
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    int turnsLeft = 3;

    public void buildTrajectories() {
//        goToCarouselFromStarting = tankDrive.trajectoryBuilder(startingPosition)
//                .back(4)
//                .build();

        goToShippingHubFromCarousel = tankDrive.trajectoryBuilder(startingPosition)
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                .forward(29)
                .build();

        goToShippingHubFromCarousel2 = tankDrive.trajectoryBuilder(goToShippingHubFromCarousel.end().plus(new Pose2d(0, 0, -Math.toRadians(125))))
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                //.lineTo(new Vector2d(shippingHubPose.getX(), shippingHubPose.getY()))
                .back(1)
                .build();

        goToAllianceFreightFromShippingHub = tankDrive.trajectoryBuilder(goToShippingHubFromCarousel2.end())
                .forward(6)
                .build();

        goToAllianceFreightFromShippingHub2 = tankDrive.trajectoryBuilder(goToAllianceFreightFromShippingHub.end().plus(new Pose2d(0, 0, -Math.toRadians(127))))
                .forward(32)
                .build();

        shippingHubAll1 = tankDrive.trajectoryBuilder(goToAllianceFreightFromShippingHub2.start())
                .back(4)
                .build();

        shippingHubAll2 = tankDrive.trajectoryBuilder(shippingHubAll1.end().plus(new Pose2d(0, 0, -Math.toRadians(-45))))
                .back(8)
                .build();

        park1 = tankDrive.trajectoryBuilder(shippingHubAll2.end())
                .forward(6)
                .build();

        park2 = tankDrive.trajectoryBuilder(park1.end().plus(new Pose2d(0, 0, -Math.toRadians(80))))
                .forward(72)
                .build();

    }

    public void switchFromMecToTank() {
        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);
        isMec = false;
    }

    public void switchFromTankToMec() {
        fl.setPosition(0.984);
        fr.setPosition(0.1);
        br.setPosition(0.955);
        bl.setPosition(0.045);
        isMec = true;
    }

    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new CVPipeline(false);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive  = new SampleTankDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotorEx.class,"caro");
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
        intakePosition = hardwareMap.get(Servo.class, "intakeLift");
        fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 &
        bl = hardwareMap.get(Servo.class, "backleft");
        // limits: 0.034 & 1
        // tank is whole numbers
        //PoseStorage.currentPose = startingPosition;
//        fl.setPosition(0.984);
//        fr.setPosition(0.1);
//        br.setPosition(0.955);
//        bl.setPosition(0.02);

        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        buildTrajectories();

        runtime.reset();
        intakePosition.setPosition(intakeUp);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("avg1 - red", pipeline.getAvg1());
            telemetry.addData("avg2 - blue", pipeline.getAvg2());
            telemetry.addData("avg3 - green", pipeline.getAvg3());
            telemetry.addData("position", pipeline.getPosition());
            telemetry.update();

            position = pipeline.getPosition();
            mecanumDrive.setPoseEstimate(startingPosition);
        }

        waitForStart();
        if (opModeIsActive()) {
            fleft.setPower(-0.1);
            fright.setPower(-0.1);
            bleft.setPower(-0.1);
            bright.setPower(-0.1);
            carousel.setPower(0.6);
            sleep(10000);
        }
    }
}
