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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

public class SetDistanceTest extends LinearOpMode {
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
    private BNO055IMU imu;
    private CRServo tapeMeasure;

    // auto things
    private double gearRatio = 30.0/20.0;
    private double tpr = 537.7 * gearRatio; // ticks per revolution
    private double wheelDiameter = 4; // inches
    private double wheelCircumference = wheelDiameter * Math.PI;
    
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        tapeMeasure = hardwareMap.get(CRServo.class, "tapeMeasure");
        
        this.setupIMU();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // move a foot
        this.pivotAngle(90, 1);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());
    }
    
    // move a set distance in inches, speed effects motor powers like setPower normally would
    public void goForwardDistance(double distance, double speed){
        double revolutions = distance / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        this.backLeft.setTargetPosition((int)ticks);
        this.backRight.setTargetPosition((int)ticks);
        this.frontLeft.setTargetPosition((int)ticks);
        this.frontRight.setTargetPosition((int)ticks);
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
    }
    
    public void pivotAngle(double angle, double speed){
        this.setupIMU();
        
        double startingOrientation = this.getYRotation();
        double finalOrientation = startingOrientation + angle;
        double extraSpeed = 0.2;
        double vgyro = this.getYRotation();
        double lastTime = 0;
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        if(finalOrientation < startingOrientation){
            this.backLeft.setPower(speed);
            this.backRight.setPower(-speed);
            this.frontLeft.setPower(speed);
            this.frontRight.setPower(-speed);
            
            while(this.getYRotation() > finalOrientation && opModeIsActive()){
                double speedFactor = 1 - Math.abs( (this.getYRotation()-startingOrientation) / finalOrientation );
                
                // small factor to prevent robot from moving slow
                speedFactor += extraSpeed;
                
                this.backLeft.setPower(speed * speedFactor);
                this.backRight.setPower(-speed * speedFactor);
                this.frontLeft.setPower(speed * speedFactor);
                this.frontRight.setPower(-speed * speedFactor);
            
                telemetry.addData("starting orientation", startingOrientation);
                telemetry.addData("final orientation", finalOrientation);
                telemetry.addData("current gyro", this.getYRotation());
                telemetry.addData("speedFactor", speedFactor);
                telemetry.addData("Velocity", this.getYVelocity().zRotationRate);
                telemetry.update();
            }
        } else {
            this.backLeft.setPower(-speed);
            this.backRight.setPower(speed);
            this.frontLeft.setPower(-speed);
            this.frontRight.setPower(speed);
            
            while(this.getYRotation() < finalOrientation && opModeIsActive()){
                double speedFactor = 1 - ( (this.getYRotation()-startingOrientation) / finalOrientation );
                
                // small factor to prevent robot from moving slow
                speedFactor += extraSpeed;
                
                this.backLeft.setPower(-speed * speedFactor);
                this.backRight.setPower(speed * speedFactor);
                this.frontLeft.setPower(-speed * speedFactor);
                this.frontRight.setPower(speed * speedFactor);
                
                telemetry.addData("starting orientation", startingOrientation);
                telemetry.addData("final orientation", finalOrientation);
                telemetry.addData("current gyro", this.getYRotation());
                telemetry.addData("speedFactor", speedFactor);
                telemetry.addData("Velocity", this.getYVelocity().zRotationRate);
                telemetry.update();
            }
        }
        
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
    }
    
    public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    
    public double getYRotation(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    
    public AngularVelocity getYVelocity(){
        return this.imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES);
    }
}
