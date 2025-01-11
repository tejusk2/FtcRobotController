package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ClawSubAssembly;
import org.firstinspires.ftc.teamcode.Deadwheel;
import org.firstinspires.ftc.teamcode.LimitSwitch;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name="Motor Tester")
public class MotorTester extends LinearOpMode {
    boolean open = true;
    //Hardwarre initializations
    boolean debug = false;
    Limelight3A limelight;
    DcMotor fl,fr,br,bl, oSlideRight, oSlideLeft, islide, enc_left;
    ServoImplEx outakeArm, outakeBox, claw, base, wrist, joint ;
    IMU imu;
    RevBlinkinLedDriver leds;
    TouchSensor left, right, intakeLeft, intakeRight;    //34.1632
    LimitSwitch oLSLeft, oLSRight, iLSLeft,iLSRight;
    ClawSubAssembly intakeArm;
    ElapsedTime transferTimer = new ElapsedTime();
    //State initialization
    enum DrivingStates {
        FULL_SPEED_MANUAL,
        LIMELIGHT,
        SLOW_SPEED_MANUAL
    }
    enum SubAssemblyStates {
        INTAKE,
        TRANSFER,
        OUTAKE,
        TRANSFER_FAILED
    }
    enum TransferStates {
        ELSEWHERE,
        TRANSFER_READY,
        SAMPLE_DROPPED
    }
    TransferStates transferState = TransferStates.ELSEWHERE;
    DrivingStates drivingState = DrivingStates.FULL_SPEED_MANUAL;
    SubAssemblyStates subState = SubAssemblyStates.INTAKE;
    Deadwheel leftX, middle, rightX;


    @Override
    public void runOpMode() throws InterruptedException {
        //more hardware init
        //Chassis Motors
        enc_left = hardwareMap.get(DcMotor.class, "enc_left");
        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        fr = hardwareMap.get(DcMotor.class, "FR");

        //servo config
        outakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outakeBox = hardwareMap.get(ServoImplEx.class, "outtakeBox");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /*
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking
        limelight.pipelineSwitch(2); // Switch to pipeline number 2
         */


        base = hardwareMap.get(ServoImplEx.class, "base");
        joint = hardwareMap.get(ServoImplEx.class, "joint");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        intakeArm = new ClawSubAssembly(limelight, base, joint, wrist, claw);

        //IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        //Linear Slides
        oSlideRight = hardwareMap.get(DcMotor.class, "slidesRight" );
        oSlideLeft = hardwareMap.get(DcMotor.class, "slides_l");
        islide = hardwareMap.get(DcMotor.class, "horSlide" );
        islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        islide.setDirection(DcMotorSimple.Direction.REVERSE);
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Limit switches
        left = hardwareMap.get(TouchSensor.class, "left");
        right = hardwareMap.get(TouchSensor.class, "right");
        intakeLeft = hardwareMap.get(TouchSensor.class, "inLeft");
        intakeRight = hardwareMap.get(TouchSensor.class, "inRight");
        oLSLeft = new LimitSwitch(left);
        oLSRight = new LimitSwitch(right);
        iLSLeft = new LimitSwitch(intakeLeft);
        iLSRight = new LimitSwitch(intakeRight);

        leftX = new Deadwheel(enc_left);
        rightX = new Deadwheel(br);
        middle = new Deadwheel(oSlideLeft);

        //Bulk caching setup
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Leds
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        leftX.reset();
        middle.reset();
        rightX.reset();
        waitForStart();

        while (opModeIsActive()){
            //chassis state machine
            fl.setPower(gamepad1.left_trigger);
            fr.setPower(gamepad1.right_trigger);
            bl.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.right_stick_y);
            //Subassembly state machine


            telemetry.addData("debug: ", debug);
            telemetry.addData("vertical slides: ", oSlideRight.getCurrentPosition());
            telemetry.addData("intake slides: ", islide.getCurrentPosition());
            telemetry.addData("out limit Left: ", oLSLeft.isPressed());
            telemetry.addData("out limit Right: ", oLSRight.isPressed());
            telemetry.addData("in limit Left: ", iLSLeft.isPressed());
            telemetry.addData("in limit Right: ", iLSRight.isPressed());

            telemetry.addData("l: ", leftX.getCurrentPosition());
            telemetry.addData("perp: ", middle.getCurrentPosition());
            telemetry.addData("r: ", rightX.getCurrentPosition());
            telemetry.update();
        }



    }
    //foundation logic
    public void gp1(){
        //DRIVETRAIN STATE LOGIC
        if (gamepad1.right_trigger > 0.1){
            drivingState = DrivingStates.SLOW_SPEED_MANUAL;
        }else{
            drivingState = DrivingStates.FULL_SPEED_MANUAL;
        }
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            imu.resetYaw();
        }
    }
    //Subassembly control logic
    public void gp2(){
        //triangle switches intake/outtake
        //ps transfers
        //right/left trigger  intake/outtake
        //cross and triangle, toggle intake servo angle(down, up)
        //left stick y - slides
        //cross - high basket
        //change to intake mode

        //if stuck trying to transfer, switch to intake mode
        if(subState == SubAssemblyStates.TRANSFER_FAILED && getRuntime() > 0.5){
            subState = SubAssemblyStates.INTAKE;
        }
        //SWITCH FROM OUTTAKE TO INTAKE
        if(subState == SubAssemblyStates.OUTAKE && gamepad2.square && getRuntime() > 0.8){
            resetRuntime();
            subState = SubAssemblyStates.INTAKE;

        }
        //SWITCH TO INTAKE FROM OUTTAKE
        if(subState == SubAssemblyStates.INTAKE && gamepad2.square && getRuntime() > 0.8){
            resetRuntime();
            subState = SubAssemblyStates.OUTAKE;
        }
        //transfer
        if(subState == SubAssemblyStates.INTAKE && gamepad2.ps && getRuntime() > 0.5){
            resetRuntime();
            transferTimer.reset();
            subState = SubAssemblyStates.TRANSFER;
            transfer();
        }
        //transfer kill switch
        if(subState == SubAssemblyStates.TRANSFER && gamepad2.triangle){
            subState = SubAssemblyStates.INTAKE;
        }


    }

    public void robotCentricPlus(double speedmult) {
        double x, y, mag, rads, rangle;
        double fieldCentricMultiplier  = 0.8;
        double rotationalMult = -0.8;
        x = -fieldCentricMultiplier * (gamepad1.left_stick_x) * speedmult;
        y = -fieldCentricMultiplier * (gamepad1.left_stick_y) * speedmult;
        double rotational = gamepad1.right_stick_x * rotationalMult * speedmult;
        mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        rads = Math.atan2(y, x);
        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        double turn = (rads < rangle) ? (Math.toRadians(360) - rangle) + (Math.abs(0 - rads)) : rads - rangle;
        //double turn = rads;
        double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
        double equationtwo = -(Math.sin(turn - (Math.PI / 4)) * mag);
        fr.setPower((equationone + rotational));
        bl.setPower((equationone - rotational));
        br.setPower((equationtwo + rotational));
        fl.setPower((equationtwo - rotational));
    }

    public void intakeServos(){
        if(gamepad2.cross && getRuntime() > 0.4){
            resetRuntime();
            if(open) {
                intakeArm.closeClaw();
                open = false;
            }else{
                intakeArm.openClaw();
                open = true;
            }
        }
        //circle scan
        if(gamepad2.circle){

        }
        if(gamepad2.dpad_up){
            intakeArm.setINTAKE_SCAN();
        }
        if(gamepad2.dpad_down){
            intakeArm.setINTAKE_PICKUP();
        }
        if(gamepad2.right_bumper && getRuntime() > 0.3){
            resetRuntime();
            wrist.setPosition(wrist.getPosition()+0.1);
        }
        if(gamepad2.left_bumper && getRuntime() > 0.3){
            resetRuntime();
            wrist.setPosition(wrist.getPosition()-0.1);
        }
        //swivel wrist

    }
    //Outtake box stuff
    public void outakeBox(){
        //high basket
        if(gamepad2.cross){
            setOUTTAKE_DROP();
        }
        //specimen
        else {
            setOUTTAKE_SPECIMEN();
        }
    }
    public void slidesLimiterIntake(double power, int upperBound){
        if((islide.getCurrentPosition() > upperBound && power > 0) || (intakeDocked() && power < 0)) {
            islide.setPower(0);
        } else{
            islide.setPower(power);
        }
    }
    public void slidesLimiterOutake(double power, int upperBound) {
        if ((-oSlideRight.getCurrentPosition() > upperBound && power > 0) && (outtakeDocked() && power < 0)){
            oSlideRight.setPower(0);
            oSlideLeft.setPower(0);

        } else {
            oSlideLeft.setPower(power);
            oSlideRight.setPower(-power);

        }
    }
    public void transfer() {
        //we need to check how long this func been running
        //transfer timer was reset before the function was called
        if(transferTimer.seconds() < 3) {
            //check if intake is in all the way, if not -- bring in
            if (intakeDocked()) {
                //check if outtake is in all the way, if not -- bring in
                if (outtakeDocked()) {
                    //transfer state starts in elsewhere
                    //reset the runtime
                    if (transferState == TransferStates.ELSEWHERE) {
                        resetRuntime();
                        transferState = TransferStates.TRANSFER_READY;
                    }
                    //set the intake to transfer, wait 0.5 seconds and open claw
                    if (transferState == TransferStates.TRANSFER_READY) {
                        intakeArm.setINTAKE_TRANSFER();
                        if (getRuntime() > 0.5) {
                            intakeArm.openClaw();
                            transferState = TransferStates.SAMPLE_DROPPED;
                        }
                    }
                    //wait 0.5 more seconds
                    if (getRuntime() > 1) {
                        if (transferState == TransferStates.SAMPLE_DROPPED) {
                            //set the intake to scan
                            intakeArm.setINTAKE_SCAN();
                            //wait 0.3 seconds
                            //go to outtake mode
                            if (getRuntime() > 1.3) {
                                setOUTTAKE_ARM_UP();
                                setOUTTAKE_SPECIMEN();
                                //switch to outtake mode
                                transferState = TransferStates.ELSEWHERE;
                                subState = SubAssemblyStates.OUTAKE;
                            }
                        }
                    }
                } else {
                    slidesLimiterOutake(-1, 1000);
                }
            } else {
                slidesLimiterIntake(-1, 1300);
            }
        }

    }
    public boolean intakeDocked(){
        if((iLSLeft.isPressed() && iLSRight.isPressed()) || islide.getCurrentPosition() <= 0){
            islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }else{
            return false;
        }
    }
    public boolean outtakeDocked(){
        if((oLSRight.isPressed() && oLSLeft.isPressed()) || -oSlideRight.getCurrentPosition() <= 0){
            oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }else{
            return false;
        }
    }
    //SERVO_POSES
    public void setOUTTAKE_ARM_DOWN(){
        outakeArm.setPosition(0);
    }
    public void setOUTTAKE_ARM_UP(){
        outakeArm.setPosition(1);
    }
    public void setOUTTAKE_TRANSFER(){
        outakeBox.setPosition(0.3);
    }
    public void setOUTTAKE_SPECIMEN(){
        outakeBox.setPosition(1);
    }
    public void setOUTTAKE_DROP(){
        outakeBox.setPosition(0);
    }




}
