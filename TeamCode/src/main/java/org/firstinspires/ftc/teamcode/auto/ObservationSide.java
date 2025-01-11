package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotDeprecated;

import java.util.List;

@Autonomous(name="Observation")
public class ObservationSide extends LinearOpMode {
    DcMotor fl,fr,br,bl, enc_left, leftSlides, rightSlides, islides;

    ServoImplEx outtakeArm, outtakeBox,base, joint, wrist, claw;

    Limelight3A limelight;
    RevBlinkinLedDriver leds;
    TouchSensor left, right, intakeLeft, intakeRight;    //34.1632

    @Override
    public void runOpMode() throws InterruptedException {
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        base = hardwareMap.get(ServoImplEx.class, "base");
        joint = hardwareMap.get(ServoImplEx.class, "joint");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        fr = hardwareMap.get(DcMotor.class, "FR");

        leftSlides = hardwareMap.get(DcMotor.class, "slides_l");

        enc_left = hardwareMap.get(DcMotor.class, "enc_left");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        rightSlides = hardwareMap.get(DcMotor.class, "slidesRight" );
        leftSlides = hardwareMap.get(DcMotor.class, "slides_l");

        islides = hardwareMap.get(DcMotor.class, "horSlide" );
        islides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
        islides.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft = hardwareMap.get(TouchSensor.class, "inLeft");
        intakeRight = hardwareMap.get(TouchSensor.class, "inRight");



        left = hardwareMap.get(TouchSensor.class, "left");
        right = hardwareMap.get(TouchSensor.class, "right");

        outtakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakeBox = hardwareMap.get(ServoImplEx.class, "outtakeBox");




        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);


        Robot robot = new Robot(fr,fl,br,bl, enc_left, leftSlides, islides, rightSlides, leds,
                outtakeArm, outtakeBox, claw, base, joint, wrist, left, right, intakeLeft, intakeRight,limelight);

        while (opModeInInit()){
            robot.setOUTTAKE_ARM_MIDDLE();
            robot.setOUTTAKE_DROP();
        }

        waitForStart();
        robot.intakeArm.setINTAKE_SCAN();
        robot.setOUTTAKE_ARM_UP_LOWERED();
        robot.setOUTTAKE_SPECIMEN();

        //Drop off preload



        robot.motionProfile(15, -24, 1);
        robot.timedCreep(1, -0.4);
        robot.setOuttakeSlides(1000);
        sleep(500);
        robot.homeOuttake();
        robot.setOUTTAKE_TRANSFER();
        robot.setOUTTAKE_ARM_TRANSFER();



        //Pickup and drop off first
        robot.motionProfile(-31, -20, 0.5);
        robot.intakeArm.setINTAKE_WALL();
        sleep(1000);
        robot.intakeArm.closeClaw();
        sleep(500);




    }

}
