/*
Copyright 2021 FIRST Tech Challenge Team FTC_13406

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class WinningOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor carousel;
    private Servo clawServoLeft;
    private Servo clawServoRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private BNO055IMU imu;
    private CRServo tapeMeasure;
    private CRServo tapeMeasureRed;
    
    private ElapsedTime globalTime;
    
    // uncomment to override default gamepad for simulated input
    //private Gamepad gamepad1 = new Gamepad();
    //private Gamepad gamepad2 = new Gamepad();
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        tapeMeasure = hardwareMap.get(CRServo.class, "tapeMeasure");
        tapeMeasureRed = hardwareMap.get(CRServo.class, "tapeMeasureRed");
        
        this.setupIMU();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        tapeMeasure.setDirection(DcMotorSimple.Direction.FORWARD);
        tapeMeasureRed.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        globalTime = new ElapsedTime();
        
        // phoenix controls temp
        int armPositionsOffset = -40; // offset for arm positions in case we adjust the arm and the initial position changes (because I don't want to change it every time)
        int armPositions[] = new int[]{-65, -125, -204, -304, -350}; // actual positions of each armPosition (armPosition treated like index)
        int armPosition = 1; // 0 - grab, 1 - lower hub, 2 - middle hub, 3 - upper hub, 4 - top
        int offset = 0; // offset to use with tuning stick
        
        // initialize arm positions according to offset
        for(int i = 0; i < armPositions.length; i++){
            armPositions[i] += armPositionsOffset;    
        }
        
        double lastClamped = 0; // last time, m,,/m,/in seconds, when the servos were TOLD to be clamped
        double clampTime = 0.3; // the amount of time to wait before allowing the servos to be clamped again
        boolean clamped = false; // whether or not servos are clamped
        
        double tapeLastSwitched = 0;
        double tapeSwitchTime = 0.3;
        boolean currentTape = true; // true = blue, false = red
        
        double startingOrientation = 0;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive Train   
            /*double movement_x;
            double movement_y;
            double movement_turn;
            
            movement_x = Range.clip(gamepad1.left_stick_x, -1, 1);
            movement_y = Range.clip(gamepad1.left_stick_y, -1, 1);
            movement_turn = Range.clip(gamepad1.right_stick_x, -1, 1);
        
            double frontLeftPower = movement_y - movement_x + movement_turn;
            double frontRightPower = movement_y + movement_x - movement_turn;
            double backLeftPower = movement_y + movement_x + movement_turn;
            double backRightPower = movement_y - movement_x - movement_turn;
            */
            
            double powerFactor = 1.3 - gamepad1.left_trigger;
            
            double movement_x = Range.clip(gamepad1.left_stick_x, -1, 1);
            double movement_y = Range.clip(-gamepad1.left_stick_y, -1, 1);
            double movement_turn = Range.clip(gamepad1.right_stick_x, -1, 1);
            
            double orientation = this.getYRotation() - startingOrientation;
            double toRadians = Math.PI / 180;
            
            telemetry.addData("orinetation", orientation);
            
            orientation *= toRadians;
            double temp = movement_x*Math.cos(orientation) + movement_y*Math.sin(orientation);
            movement_y = -movement_x*Math.sin(orientation) + movement_y*Math.cos(orientation);
            movement_x = temp;
            
            double fl = movement_y + movement_turn + movement_x;
            double fr = movement_y - movement_turn - movement_x;
            double bl = movement_y + movement_turn - movement_x;
            double br = movement_y - movement_turn + movement_x;
            
            backLeft.setPower(bl*powerFactor);
            frontLeft.setPower(fl*powerFactor);
            frontRight.setPower(fr*powerFactor);
            backRight.setPower(br*powerFactor);
            
            // realign FOD
            if(gamepad1.right_bumper){
                startingOrientation = this.getYRotation();
            }
            
            //Carousel
            double carouselPower = Range.clip(gamepad1.right_trigger, 0, .7);
            double carouselDirection = currentTape ? 1 : -1;
            carousel.setPower(carouselPower*carouselDirection);
            
            // Telemetry:
            /*telemetry.addData("Target Back Left Motor Power", backLeftPower);
            telemetry.addData("Target Front Left Motor Power", frontLeftPower);
            telemetry.addData("Target Front Right Motor Power", frontRightPower);
            telemetry.addData("Target Back Right Motor Power", backRightPower);
            
            telemetry.addData("Back Right Motor Power", backLeft.getPower());
            telemetry.addData("Front Right Motor Power", frontLeft.getPower());
            telemetry.addData("Front Left Motor Power", frontRight.getPower());
            telemetry.addData("Back Left Motor Power", backRight.getPower());
            */
            
            telemetry.addData("Status", "Running");
            
            // phoenix controls
            double movementArm = Range.clip(gamepad2.left_stick_y, -1, 1);
            double movementTune = Range.clip(gamepad2.right_stick_y, -1, 1);
            
            // move claw to default positions
            if(movementArm != 0){
                // only move if arm is not currently moving (to prevent buildup)
                if(!armMotor.isBusy()){
                    // reset offset
                    //offset = 0;
                    
                    // move arm 
                    if(movementArm > 0) armPosition = Math.max(0, --armPosition);
                    if(movementArm < 0) armPosition = Math.min(armPositions.length-1, ++armPosition);
                }
            } else {
                // adjust to resting position if grabbing, because we don't want the robot to run over its own arm
                if(armPosition == 0) armPosition = 1;
            }
            
            // use tuning position
            if(movementTune != 0){
                // offset arm
                offset += movementTune;
            }
            
            // set the motor to position
            armMotor.setTargetPosition(armPositions[armPosition] + offset);
            armMotor.setPower(0.75);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            telemetry.addData("arm position #", armMotor.getTargetPosition());
            
            // check if servos should be clamped
            if(gamepad2.a){
                // if servos inactive, clamp
                if(globalTime.seconds() >= lastClamped+clampTime){
                    lastClamped = globalTime.seconds();
                    clamped = !clamped;
                }
            }
            
            // check if tape should be switched
            if(gamepad1.a){
                if(globalTime.seconds() > tapeLastSwitched+tapeSwitchTime){
                    tapeLastSwitched = globalTime.seconds();
                    (currentTape ? tapeMeasure : tapeMeasureRed).setPower(0.0); // reset servos
                    currentTape = !currentTape; // switch tape
                }
            }
            
            // clamp servos
            if(!clamped){
                this.unclamp();
            } else {
                this.clamp();
            }
            
            // tape measure
            CRServo currentTapeObject = currentTape ? tapeMeasure : tapeMeasureRed;
            
            // log to telemetry
            if(currentTape){
                telemetry.addData("Current tape", "blue");
            } else {
                telemetry.addData("Current tape", "red");
            }
            
            if(gamepad1.y){
                currentTapeObject.setPower(-1.0);
            } else if(gamepad1.b){
                currentTapeObject.setPower(1.0);
            } else if(gamepad1.x){
                currentTapeObject.setPower(0.0);
            }
            
            telemetry.addData("armPosition", armPosition);
            
            telemetry.update();
        }
    }
    
    public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    
    public double getYRotation(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    
    public void clamp(){
        clawServoLeft.setPosition(0.4);
        clawServoRight.setPosition(0.4);
    }
    
    public void unclamp(){
        clawServoLeft.setPosition(0.0);
        clawServoRight.setPosition(0.9);
    }
}
