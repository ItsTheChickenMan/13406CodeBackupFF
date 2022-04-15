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
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import AutoTools.EzrasLaw;

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
@Autonomous

public class PhoenixAutoBlueDuck extends EzrasLaw {
    /*private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotorEx armMotor;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor carousel;
    private Servo clawServoLeft;
    private Servo clawServoRight;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private BNO055IMU imu;
    */
    
/* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
      "Duck",
      "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AcnDHVv/////AAABmRnrh4YWCUq9oXJiDJfQ4tpGcyqNmoiVhQEAVeBCc1pxc59UM914g/AfIlzXvB+DEj61Op1AD7CdC/shuEqyF7E9jn4nYrzRhWXBB2ZZOISZ7dsulJ3ZxGsbSF6xpAuvnGoVTZL2wZGVzFvHmz7re2Ek7IH7Rc59kzFI2mkvtUxHPU4hqR14fXBNvHG5bdVoidyZVm50h5nyDdbnOij/2sJXYh01NzH02y2O1voPzA2XT6sEKAgqi9j8UQ6q4XP/wSa1BLPwwcInLnEJohQSOED4GGWW+M9WHZ6t2zUkSge+YPzBINQwczICkTG6qfDwlRKb7ghKYjvScxr2quuUnIGZntALNrHSlP5OQlBW7QHf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    
    // AutoTools stuff
    private ElapsedTime globalTime;
    
    private double gearRatio = 30.0/20.0;
    private double tpr = 537.7 * gearRatio; // ticks per revolution
    private double wheelDiameter = 4; // inches
    private double wheelCircumference = wheelDiameter * Math.PI;
    private double strafeMultiplier = 12.0/11.0;
    private double autoSpeed = 0.5; // relative speed of each movement in auto (1.0 is max, 0.0 is none at all)
    private double intendedRotation = 0; // what the rotation of the bot SHOULD be
    
    private double x, y; // relative x and y to where the bot starts (potentially innacurate due to braking/skidding)
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        this.setupIMU();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // timer is used in initialization a bit so start it up here (will be reset later)
        globalTime = new ElapsedTime();
        
        // unclamp the motors to give a brief period to place the pre-loaded block (should be enough time during vurofia/tfod initialization to do so)
        this.unclamp();
        
        //drive = new Drive(this, imu, frontLeft, frontRight, backLeft, backRight);
        //globalTime = new ElapsedTime();
        
        /* initialize vuforia & tfod... */
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        
        // initialize arm
        int armPositionsOffset = 1075; // offset for arm positions in case we adjust the arm and the initial position changes (because I don't want to change every position every time)
        int armPositions[] = new int[]{-65, -107, -204, -315, -350}; // actual positions of each armPosition (armPosition treated like index)
        int armPosition = 0;
        
        // initialize arm positions according to offset
        for(int i = 0; i < armPositions.length; i++){
            armPositions[i] += armPositionsOffset;    
        }
        
        // make sure we've waited for at least 5 seconds
        while(this.globalTime.seconds() < 5){
            telemetry.addLine("Clamping in " + Math.round(5-this.globalTime.seconds()) + " seconds...");
            telemetry.update();
        };
        
        // reclamp claw to grab preloaded block
        this.clamp();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // settings (the boolean variables are only for default initialization, use the settings array to access them in the program)
        boolean parkInWarehouse = false;
        
        // settings index
        int PARK_IN_WAREHOUSE = 0;
        
        String[] settingsStrings = {"Park In Warehouse?"};
        boolean[] settings = {parkInWarehouse};
        int settingIndex = 0;
        
        // bools to make buttons work properly
        boolean toggledLastFrame = false;
        boolean switchedLastFrame = false;
        
        while(!isStarted()){
            // inform user of settings
            telemetry.addLine("Press A to toggle selected setting");
            telemetry.addLine("Press left or right bumpers to switch selected setting");
            telemetry.addLine("");
            telemetry.addData("Selected setting", settingsStrings[settingIndex]);
            telemetry.addLine("");
            telemetry.addLine("Current settings:");
            
            for(int i = 0; i < settingsStrings.length; i++){
                telemetry.addData(settingsStrings[i], settings[i]);
            }
            
            telemetry.update();
            
            if(gamepad1.a && !toggledLastFrame){
                settings[settingIndex] = !settings[settingIndex];
                toggledLastFrame = true;
            } else if(!gamepad1.a){
                toggledLastFrame = false;
            }
            
            if(gamepad1.left_bumper && !switchedLastFrame){
                settingIndex = settingIndex-1 < 0 ? settings.length-1 : settingIndex-1;
                switchedLastFrame = true;
            } else if(gamepad1.right_bumper && !switchedLastFrame){
                settingIndex = settingIndex+1 >= settings.length ? 0 : settingIndex+1;
                switchedLastFrame = true;
            } else if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                switchedLastFrame = false;
            }
        }
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // initialize timer at start so time isn't changed by the initialization of vuforia/tfod
        globalTime = new ElapsedTime();
        
        // move arm out of the way
        /*armPosition = 1;
        
        armMotor.setTargetPosition(armPositions[armPosition]);
        armMotor.setPower(0.25);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        
        // go to the carousel
        this.strafeDistance(4, 0.75, true);
        this.goForwardDistance(-2.5, 0.75, true);
        
        // spin da wheel
        carousel.setPower(0.6);
        
        // (for 3 seconds)
        double startTime = this.globalTime.seconds();
        
        while(this.globalTime.seconds() <= startTime+2.65 && opModeIsActive());
        
        // stop spinnin
        carousel.setPower(0.0);
        
        // strafe towards bar codes
        this.strafeDistance(5, 0.65, true);
        
        // move to first barcode
        this.goForwardDistance(9, 0.5, true);
        
        // wait for detection (or for 2.5 seconds)
        boolean rightSide = this.waitForDuck(2.5);
        
        boolean middleSide = false;
        
        if(!rightSide){
            // move to next barcode
            this.goForwardDistance(8.25, 0.5, true);
        
            // wait for duck
            middleSide = this.waitForDuck(2.5);
        }
        
        // if detected at all, move normally
        if(rightSide){
            // move the rest of the distance to hub
            this.goForwardDistance(16.75, 0.8, true);
        
            // strafe to hub
            this.strafeDistance(13, 0.8);
        } else {
            // move back a slight bit
            this.goForwardDistance(-10.0, 1.0, true);
            
            // strafe to roughly parallel with hub
            this.strafeDistance(29, 0.8, true);
        }
        
        int hubLevel = rightSide ? 0 : middleSide ? 1 : 2;
        
        // tell arm to move to position (can do this while driving)
        armPosition = 3-hubLevel;
        
        // set the motor to position
        armMotor.setTargetPosition(armPositions[armPosition]);
        armMotor.setPower(0.75);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // wait for arm to finish
        while(armMotor.isBusy() && opModeIsActive());
        
        if(rightSide){
            // rotate slightly to line up with hub
            this.pivotAngle(-50, 0.7);
        } else {
            /*double distance;
            
            // move a different distance to hub depending on hub level
            // these values are usually inconsistent,
            if(middleSide){
                distance = 12;
            } else {
                distance = 12;
            }*/
            
            // move rest of the distance to hub
            this.goForwardDistance(12, 1.0, true);
        }
        
        String hubLevelString = hubLevel == 2 ? "Bottom" : hubLevel == 1 ? "Middle" : "Top";
        
        telemetry.addData("Hub Level", hubLevelString);
        telemetry.update();
        
        this.unclamp();
        
        // wait a sec to avoid skidding
        startTime = this.globalTime.seconds();
        while(this.globalTime.seconds() < startTime+1 && opModeIsActive());
        
        // move backward a bit to prevent the arm hitting the hub when it stops
        this.goForwardDistance(-6, 1.0, true);
        
        // move arm the back
        armMotor.setTargetPosition(100);
        armMotor.setPower(0.5);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // once arm is done moving, disable arm
        startTime = this.globalTime.seconds();
        
        while((armMotor.isBusy() || this.globalTime.seconds() < startTime+1) && opModeIsActive());
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        if(!rightSide){
            // move backward to avoid the duck
            this.goForwardDistance(-14, 0.5, true);
            // go into the parking spot
            this.goToCoordinates(24.5, -9, 0.5, true);
        } else {
            // go into the parking spot
            this.goToCoordinates(24.5, -9, 0.5, false);
        }
        
        while(this.opModeIsActive()){
            telemetry.addData("x", this.x);
            telemetry.addData("y", this.y);
            telemetry.update();
        };
    }
    
    /*public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    
    // set single power for each motor
    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }
    
    // set individual powers for motors
    public void setPowers(double fl, double fr, double bl, double br){
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }
    
    // move forward (alias for setPower)
    // direction defined by sign of power, - = backwards & + = positive
    public void forward(double power){
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        this.setPower(power);
    }
    
    // strafe
    // direction defined by sign of power, - = left & + = right
    public void strafe(double power){
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        this.setPowers(power, -power, -power, power);
    }
    
    // turn
    // direction defined by sign of power, - = left & + = right
    public void turn(double power){
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        this.setPowers(power, -power, power, -power);
    }
    
    // stop
    public void stopMotors(){
        this.setPower(0);    
    }
    
    // testing versions of the above functions which don't do anything, effectively disabling the motors
    
    // set single power for each motor
    public void setPower(double power){
        return;
    }
    
    // set individual powers for motors
    public void setPowers(double fl, double fr, double bl, double br){
        return;
    }
    
    // move forward (alias for setPower)
    // direction defined by sign of power, - = backwards & + = positive
    public void forward(double power){
        return;
    }
    
    // strafe
    // direction defined by sign of power, - = left & + = right
    public void strafe(double power){
        return;
    }
    
    // turn
    // direction defined by sign of power, - = left & + = right
    public void turn(double power){
        return;
    }
    
    // stop
    public void stopMotors(){
        return;
    }
    
    private void initVuforia() {
        
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
    public void unclamp(){
        clawServoLeft.setPosition(0.0);
        clawServoRight.setPosition(0.83);
    }
    
    public void clamp(){
        clawServoLeft.setPosition(0.4);
        clawServoRight.setPosition(0.4);
    }
    
    // move a set distance in inches, speed affects motor powers like setPower normally would
    public void goForwardDistance(double distance, double speed){
        // change speed relative to auto
        speed *= autoSpeed;
        
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
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
        
        // TODO: use starting orientation
        this.x += Math.sin(this.toRadians(-this.intendedRotation))*distance;
        this.y += Math.cos(this.toRadians(-this.intendedRotation))*distance;
        
        telemetry.addData("x", this.x);
        telemetry.addData("y", this.y);
        telemetry.addData("orientation", -this.intendedRotation);
        telemetry.update();
    }
    
    // move a set distance in inches, speed affects motor powers like setPower normally would
    public void goForwardDistance(double distance, double speed, boolean blocking){
        // change speed relative to auto
        speed *= autoSpeed;
        
        double startingOrientation = this.getYRotation();
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
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
        
        // wait until done if blocking
        if(blocking){
            while(opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy());
        }
        
        double rotation = -this.intendedRotation;
        
        // if the actual rotation differs by more than 3.5 degrees, use actual instead
        if(Math.abs(this.getYRotation() - this.intendedRotation) > 3.5){
            rotation = -this.getYRotation();
        }
        
        this.x += Math.sin(this.toRadians(rotation))*distance;
        this.y += Math.cos(this.toRadians(rotation))*distance;
        
        telemetry.addData("x", this.x);
        telemetry.addData("y", this.y);
        telemetry.addData("orientation used", rotation);
        telemetry.update();
    }
    
    public void strafeDistance(double distance, double speed){
        // change speed relative to auto
        speed *= autoSpeed;
        
        double revolutions = distance / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        ticks *= this.strafeMultiplier;
        
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.backLeft.setTargetPosition(-(int)ticks);
        this.backRight.setTargetPosition((int)ticks);
        this.frontLeft.setTargetPosition((int)ticks);
        this.frontRight.setTargetPosition(-(int)ticks);

        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
        
        // TODO: use starting orientation
        // pretty much don't use the non-blocking versions anyways so I'm not too worried about it
        this.x += Math.cos(this.toRadians(-this.intendedRotation))*distance;
        this.y += Math.sin(this.toRadians(-this.intendedRotation))*distance;
        
        telemetry.addData("x", this.x);
        telemetry.addData("y", this.y);
        telemetry.addData("orientation", -this.intendedRotation);
        telemetry.update();
    }
    
    public void strafeDistance(double distance, double speed, boolean blocking){
        // change speed relative to auto
        speed *= autoSpeed;
        
        double startingOrientation = this.getYRotation();
        double revolutions = distance / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        ticks *= this.strafeMultiplier;
        
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.backLeft.setTargetPosition(-(int)ticks);
        this.backRight.setTargetPosition((int)ticks);
        this.frontLeft.setTargetPosition((int)ticks);
        this.frontRight.setTargetPosition(-(int)ticks);

        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        this.backLeft.setPower(speed);
        this.backRight.setPower(speed);
        this.frontLeft.setPower(speed);
        this.frontRight.setPower(speed);
        
        if(blocking){
            while(opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy());
        }
        
        double rotation = -this.intendedRotation;
        
        // if the actual rotation differs by more than 3.5 degrees, use actual instead
        if(Math.abs(this.getYRotation() - this.intendedRotation) > 3.5){
            rotation = -this.getYRotation();
        }
        
        this.x += Math.cos(this.toRadians(rotation))*distance;
        this.y += Math.sin(this.toRadians(rotation))*distance;
        
        telemetry.addData("x", this.x);
        telemetry.addData("y", this.y);
        telemetry.addData("orientation used", rotation);
        telemetry.update();
    }
    
    public void pivotAngle(double angle, double speed){
        this.setupIMU();
        
        double startingOrientation = this.getYRotation();
        double finalOrientation = startingOrientation + angle;
        
        // flip final orientation if beyond imu bounds
        if(finalOrientation > 180 || finalOrientation < -180){
            finalOrientation = 360 - finalOrientation;
        }
        
        this.intendedRotation = finalOrientation;
        
        double extraSpeed = 0.25;
        
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
                telemetry.update();
            }
        } else {
            this.backLeft.setPower(-speed);
            this.backRight.setPower(speed);
            this.frontLeft.setPower(-speed);
            this.frontRight.setPower(speed);
            
            while(this.getYRotation() < finalOrientation && opModeIsActive()){
                double speedFactor = 1 - Math.abs( (this.getYRotation()-startingOrientation) / finalOrientation );
                
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
                telemetry.update();
            }
        }
        
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
    }
    
    public void goToCoordinates(double dx, double dy, double speed){
        // get unclamped rotation
        double rotation = this.intendedRotation;
        
        // normalize between -360 and 360
        while(rotation > 360 || rotation < -360 && opModeIsActive()){
            rotation += -Math.signum(rotation)*360;   
        }
        
        // convert >180 or <-180 to rotation in opposite direction
        if(rotation > 180){
            rotation = -360 + rotation;
        } else if(rotation < -180){
            rotation = 360 + rotation;
        }
        
        // rotate to 0
        if(rotation != 0){
            this.pivotAngle(-rotation, 0.5);
        }
        
        // figure out x and y
        double forwardDistance = dy - this.y;
        double strafeDistance = dx - this.x;
        
        this.goForwardDistance(forwardDistance, speed, true);
        this.strafeDistance(strafeDistance, speed, true);
    }
    
    // go to coordinates (and strafe first or not)
    public void goToCoordinates(double dx, double dy, double speed, boolean strafeFirst){
        // get unclamped rotation
        double rotation = this.intendedRotation;
        
        // normalize between -360 and 360
        while(rotation > 360 || rotation < -360 && opModeIsActive()){
            rotation += -Math.signum(rotation)*360;   
        }
        
        // convert >180 or <-180 to rotation in opposite direction
        if(rotation > 180){
            rotation = -360 + rotation;
        } else if(rotation < -180){
            rotation = 360 + rotation;
        }
        
        // rotate to 0
        if(rotation != 0){
            this.pivotAngle(-rotation, 0.5);
        }
        
        // figure out x and y
        double forwardDistance = dy - this.y;
        double strafeDistance = dx - this.x;
        
        if(!strafeFirst){
            this.goForwardDistance(forwardDistance, speed, true);
            this.strafeDistance(strafeDistance, speed, true);
        } else {
            this.strafeDistance(strafeDistance, speed, true);
            this.goForwardDistance(forwardDistance, speed, true);
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
    
    // look for a duck in the camera until one is found, or until time is > maxTime 
    public boolean waitForDuck(double maxTime){
        // wait for detection (or for 2.5 seconds)
        double startTime = this.globalTime.seconds();
        
        boolean detected = false;
        
        while(this.globalTime.seconds() < startTime + maxTime && !detected && opModeIsActive()){
            // check for recognitions
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                
                if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
                    Recognition recognition;

                    // make sure recognition is a duck
                    int i = 0;
                    boolean found = false;
                    while(i < updatedRecognitions.size() && opModeIsActive()){
                        recognition = updatedRecognitions.get(i);
                        
                        telemetry.addData("Recognition label", recognition.getLabel());
                        
                        if(recognition.getLabel().equals("Duck")){
                            detected = true;
                            break;
                        }
                        
                        i++;
                    }
                }
                
                telemetry.update();
                
                if(detected){
                    break;
                }
            }
        }
        
        return detected;
    }
    
    public double toRadians(double degrees){
        return degrees * Math.PI / 180;
    }
    
    public double toDegrees(double radians){
        return radians * 180 / Math.PI;
    }*/
}
