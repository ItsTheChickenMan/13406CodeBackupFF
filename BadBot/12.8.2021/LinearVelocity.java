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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
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

public class LinearVelocity extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor carousel;
    private Servo clawServoLeft;
    private Servo clawServoRight;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private Gyroscope imu;
    private CRServo tapeMeasure;
    private CRServo tapeMeasureRed;

    private ElapsedTime globalTime;
    
    private double gearRatio = 30.0/20.0;
    private double tpr = 537.7 * gearRatio; // ticks per revolution
    private double wheelDiameter = 4; // inches
    private double wheelCircumference = wheelDiameter * Math.PI;
    private double strafeMultiplier = 12.0/11.0;
    private double autoSpeed = 0.5; // relative speed of each movement in auto (1.0 is max, 0.0 is none at all)
    private double unclampedRotation = 0;
    
    private double x, y; // relative x and y to where the bot starts (potentially innacurate due to braking/skidding)
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        tapeMeasure = hardwareMap.get(CRServo.class, "tapeMeasure");
        tapeMeasureRed = hardwareMap.get(CRServo.class, "tapeMeasureRed");
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        globalTime = new ElapsedTime();
        
        this.goForwardDistance(24*2, 24, 1.5, 1.5);
        //this.goForwardDistanceOld(24*2, 48);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
    
    // gaussian velocity w/go set distance
    // distance = inches, maxVelocity = inches/second
    public void goForwardDistance(double distance, double maxVelocity, double accelerationTime, double deccelerationTime){
        double revolutions = distance / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double startTime = this.globalTime.seconds();
        
        // coefficient when accelerating
        double positiveCoefficient = 1 / accelerationTime;
        
        // coefficient when deccelerating
        double negativeCoefficient = 1 / deccelerationTime;
        
        // max velocity in ticks
        double actualVelocity = (maxVelocity / this.wheelCircumference) * this.tpr;
        
        // how far the bot goes when accelerating
        double accelerationDistance = (actualVelocity/2)*accelerationTime;
        // ditto for deccelerating
        double deccelerationDistance = (actualVelocity/2)*deccelerationTime;
        
        telemetry.addData("accelerationDistance", accelerationDistance);
        telemetry.addData("ticks", ticks);
        telemetry.addData("(ticks-accelerationDistance)/actualVelocity", (ticks-accelerationDistance)/actualVelocity);
        
        // time to start deccelerating
        double deccelerationPoint = (ticks-accelerationDistance)/actualVelocity - deccelerationTime;
        
        double totalTime = accelerationTime + deccelerationTime + ((ticks-accelerationDistance-deccelerationDistance)/actualVelocity - deccelerationTime - deccelerationTime);
        
        // if the time to accelerate and move the full distance is less than decceleration time
        if((ticks-accelerationDistance)/actualVelocity < deccelerationTime){
            // set the deccelerationPoint to half the time it takes
            deccelerationPoint = (ticks-accelerationDistance)/actualVelocity/2;
        }
        
        telemetry.addData("deccelerationPoint", deccelerationPoint);
        telemetry.update();
        
        while(Math.abs(this.frontLeft.getCurrentPosition()) < Math.abs(ticks) && Math.abs(this.frontRight.getCurrentPosition()) < Math.abs(ticks) && Math.abs(this.backLeft.getCurrentPosition()) < Math.abs(ticks) && Math.abs(this.backRight.getCurrentPosition()) < Math.abs(ticks) && opModeIsActive()){
            // I called the file gaussian velocity, but we're just going to use a quadratic curve to accelerate
            double elapsed = this.globalTime.seconds() - startTime;
            double velocityFactor;
            
            if(this.globalTime.seconds() > startTime+deccelerationPoint && opModeIsActive()){
                telemetry.addLine("deccelerating...");
                telemetry.update();
                
                velocityFactor = -negativeCoefficient * elapsed + totalTime;
            } else {
                velocityFactor = positiveCoefficient * elapsed;
            }
            
            velocityFactor = velocityFactor > 1.0 ? 1.0 : velocityFactor;
            
            this.backLeft.setVelocity(actualVelocity * velocityFactor);
            this.backRight.setVelocity(actualVelocity * velocityFactor);
            this.frontLeft.setVelocity(actualVelocity * velocityFactor);
            this.frontRight.setVelocity(actualVelocity * velocityFactor);
        }
        
        this.frontLeft.setVelocity(0);
        this.frontRight.setVelocity(0);
        this.backLeft.setVelocity(0);
        this.backRight.setVelocity(0);
        
        // use normal setTargetPosition to do correction
        int frontLeftCorrection = (int)ticks;
        int frontRightCorrection = (int)ticks;
        int backLeftCorrection = (int)ticks;
        int backRightCorrection = (int)ticks;
        
        // correction
        this.frontLeft.setTargetPosition(frontLeftCorrection);
        this.frontRight.setTargetPosition(frontRightCorrection);
        this.backLeft.setTargetPosition(backLeftCorrection);
        this.backRight.setTargetPosition(backRightCorrection);
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        this.frontLeft.setVelocity(actualVelocity);
        this.frontRight.setVelocity(actualVelocity);
        this.backLeft.setVelocity(actualVelocity);
        this.backRight.setVelocity(actualVelocity);
        
        // wait until done correcting
        while(this.frontLeft.isBusy() && this.frontRight.isBusy() && this.backLeft.isBusy() && this.backRight.isBusy() && opModeIsActive());
    }
    
    public void goForwardDistanceOld(double distance, double maxVelocity){
        // change speed relative to auto
        //speed *= autoSpeed;
        
        double revolutions = distance / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.backLeft.setTargetPosition((int)ticks);
        this.backRight.setTargetPosition((int)ticks);
        this.frontLeft.setTargetPosition((int)ticks);
        this.frontRight.setTargetPosition((int)ticks);

        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        // max velocity in ticks
        double speed = (maxVelocity / this.wheelCircumference) * this.tpr;
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
        
        //this.x += Math.sin(this.toRadians(-this.getYRotation()))*distance;
        //this.y += Math.cos(this.toRadians(-this.getYRotation()))*distance;
    }
}
