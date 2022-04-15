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

public class PhoenixAutoRedWarehouse extends LinearOpMode {
    private Blinker control_Hub;
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
        int armPositionsOffset = 1145; // offset for arm positions in case we adjust the arm and the initial position changes (because I don't want to change every position every time)
        int armPositions[] = new int[]{-65, -107, -204, -304, -350}; // actual positions of each armPosition (armPosition treated like index)
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
        
        // move along barcodes
        double actionTime = 5; // total amount of time to strafe for
        double endTime = this.globalTime.seconds() + actionTime; // get start time
        this.goForwardDistance(16, 0.1);
        
        // duck detection constants
        double rightSide = 5.5; // if detection is less than this time, assume duck is right side
        double middleSide = 3.75; // if detection is less than this time (but not less than previous time) assume duck is middle
        double leftSide = 1.5;
        // if duck is not detected or greater than both of those, assume duck is left
        double times[] = {rightSide, middleSide};
        
        // total number of detections to get before no longer detecting and using the last detection for time
        int maxDetections = Integer.MAX_VALUE; // permit a lot more detections to get around seeing a duck way too early sometimes
        int totalDetections = 0;
        int hubLevel = 2; // 0 = top (left), 1 = middle, 2 = bottom (right) (default)
        boolean usedDefault = true; // debugging
        // TODO: didn't fact check these hub levels, might need to be reversed
        
        // run while moving (check if motors are busy, for now just checks one but later I will check all drive motors)
        while (this.backLeft.isBusy() && opModeIsActive()){
            // check for recognitions
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                
                if (updatedRecognitions != null && updatedRecognitions.size() != 0 && totalDetections < maxDetections) {
                    double detectionTime = this.globalTime.seconds();
                    
                    Recognition recognition;

                    // make sure recognition is a duck
                    int i = 0;
                    boolean found = false;
                    while(i < updatedRecognitions.size()){
                        recognition = updatedRecognitions.get(i);
                        
                        if(recognition.getLabel().equals("Duck")){
                            telemetry.addData("Recognition label: ", recognition.getLabel());
                            telemetry.addData("Detection time", detectionTime);
                            
                            // only increment total detections if detection is a duck
                            totalDetections++;
                            
                            found = true;
                            break;
                        }
                        
                        i++;
                    }
                    
                    if(found){
                        // see if detection is right side
                        // TODO: improve this so it isn't a chain of if/else
                        if(detectionTime < leftSide){
                            hubLevel = 2;
                        } else if(detectionTime < middleSide){
                            hubLevel = 1;
                        } else if(detectionTime < rightSide){ // see if detection is middle
                            hubLevel = 0;
                        } // no need for another check since it default to left
                        
                        usedDefault = false;
                    }
                }
                
                telemetry.update();
            }
        }
        
        this.stopMotors();
        
        // tell arm to move to position (can do this while driving)
        armPosition = 3-hubLevel;
        
        // set the motor to position
        armMotor.setTargetPosition(armPositions[armPosition]);
        armMotor.setPower(0.75);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        // move to shipping hub
        this.pivotAngle(-90, 0.9);
        this.goForwardDistance(14, 1.0);
        
        while (this.backLeft.isBusy() && opModeIsActive());
        
        this.stopMotors();
        
        // wait for arm to finish
        while(armMotor.isBusy());
        
        // rotate slightly to line up with hub
        this.pivotAngle(20, 0.9);
        
        while (this.backLeft.isBusy() && opModeIsActive());
        
        this.stopMotors();
        
        // move forward a certain distance depending on hub level (needs to be closer for top level)
        // program this later
        
        String hubLevelString = hubLevel == 2 ? "Bottom" : hubLevel == 1 ? "Middle" : "Top";
        
        telemetry.addData("Hub Level", hubLevelString);
        telemetry.addData("Used Default", usedDefault);
        telemetry.update();
        
        this.unclamp();
        
        // move backward a bit to prevent the arm hitting the hub when it stops
        this.goForwardDistance(-6, 1.0);
        
        while(this.opModeIsActive());
    }
    
    /*public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }*/
    
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
    
    /*
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
    */

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
        clawServoLeft.setPosition(0.25);
        clawServoRight.setPosition(0.8);
    }
    
    public void clamp(){
        clawServoLeft.setPosition(0.6);
        clawServoRight.setPosition(0.35);
    }
    
    // move a set distance in inches, speed effects motor powers like setPower normally would
    public void goForwardDistance(double distance, double speed){
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
    }
    
    public void pivotAngle(double angle, double speed){
        this.setupIMU();
        
        double startingOrientation = this.getYRotation();
        double finalOrientation = startingOrientation + angle;
        double extraSpeed = 0.2;
        
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
    
    public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    
    public double getYRotation(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
