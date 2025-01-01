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

@Autonomous(name="basket")
public class BasketSide extends LinearOpMode {
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
            robot.intakeArm.setINTAKE_SCAN();
        }

        waitForStart();

        //Drop off preload
        robot.motionProfile(-15, 8, 0.5);
        robot.setOUTTAKE_ARM_UP();
        sleep(3000);
        robot.setOUTTAKE_ARM_TRANSFER();

        //Pickup and drop off first
        robot.motionProfile(9.75, 14, 0.5);
        robot.intakeArm.setINTAKE_PICKUP();
        sleep(500);
        robot.intakeArm.closeClaw();
        sleep(300);
        robot.delayedFunc(0, ()->{robot.transfer();});
        robot.motionProfile(-10, -14, 0.5);
        robot.setOUTTAKE_ARM_UP();
        sleep(3000);
        robot.setOUTTAKE_ARM_TRANSFER();

        //pickup and drop off second
        robot.motionProfile(5, 14, 0.5);
        robot.intakeArm.setINTAKE_PICKUP();
        sleep(500);
        robot.intakeArm.closeClaw();
        sleep(300);
        robot.delayedFunc(0, ()->{robot.transfer();});
        robot.motionProfile(-5, -14, 0.5);
        robot.setOUTTAKE_ARM_UP();
        sleep(3000);
        robot.setOUTTAKE_ARM_TRANSFER();

        //pickup and drop off third
        robot.motionProfile(3, 4, 0.5);
        robot.rotateWithPID(45);
        robot.intakeArm.wrist.setPosition(0.2);
        robot.motionProfile(5.5, 8.5, 0.5);
        robot.intakeArm.setINTAKE_PICKUP();
        sleep(500);
        robot.intakeArm.closeClaw();
        sleep(300);
        robot.delayedFunc(0, ()->{robot.transfer();});
        robot.rotateWithPID(-45);
        robot.motionProfile(-5.5, -8.5, 0.5);
        robot.setOUTTAKE_ARM_UP();
        sleep(3000);





    }

}
