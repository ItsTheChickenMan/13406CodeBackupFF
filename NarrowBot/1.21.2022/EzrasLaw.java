/*
Copyright 2022 FIRST Tech Challenge Team FTC_13406

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

package AutoTools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.Frame;
import com.qualcomm.robotcore.hardware.CRServo;
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

public abstract class EzrasLaw extends LinearOpMode {
    public Blinker control_Hub;
    public Blinker expansion_Hub_2;
    public HardwareDevice webcam_1;
    public DcMotorEx armMotor;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx carousel;
    public Servo clawServoLeft;
    public Servo clawServoRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public BNO055IMU imu;
    public CRServo tapeBlue;
    public CRServo tapeRed;

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
    public static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/FreightFrenzy_PIPE.tflite";
    public static final String[] LABELS = {
      "Pipe"
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
    public static final String VUFORIA_KEY =
            "AcnDHVv/////AAABmRnrh4YWCUq9oXJiDJfQ4tpGcyqNmoiVhQEAVeBCc1pxc59UM914g/AfIlzXvB+DEj61Op1AD7CdC/shuEqyF7E9jn4nYrzRhWXBB2ZZOISZ7dsulJ3ZxGsbSF6xpAuvnGoVTZL2wZGVzFvHmz7re2Ek7IH7Rc59kzFI2mkvtUxHPU4hqR14fXBNvHG5bdVoidyZVm50h5nyDdbnOij/2sJXYh01NzH02y2O1voPzA2XT6sEKAgqi9j8UQ6q4XP/wSa1BLPwwcInLnEJohQSOED4GGWW+M9WHZ6t2zUkSge+YPzBINQwczICkTG6qfDwlRKb7ghKYjvScxr2quuUnIGZntALNrHSlP5OQlBW7QHf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    
    // AutoTools stuff
    public ElapsedTime globalTime;
    
    public double gearRatio = 30.0/20.0;
    public double tpr = 537.7 * gearRatio; // ticks per revolution
    public double wheelDiameter = 4; // inches
    public double wheelCircumference = wheelDiameter * Math.PI;
    public double strafeMultiplier = 12.0/11.0;
    public double autoSpeed = 0.5; // relative speed of each movement in auto (1.0 is max, 0.0 is none at all)
    public double startingOrientation = 0.0; // hard set starting orientation of the bot relative to the blue auto duck orientation (0)
    public double intendedRotation = 0; // what the rotation of the bot SHOULD be
    
    public double x, y; // relative x and y to where the bot starts (potentially innacurate due to braking/skidding)
    
    public String[] finalMessages = {"mission accomplished", "payload delivered", "job well done", "pheonix is a genius"};
    
    // global armPositions (relative to starting from the back)
    public int[] armPositions = {1062, 1000, 888, 784, 714};
    public int armOffset = 0; // necessary because the arm has a hard time reaching exact positions sometimes
    
    /*public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }*/
    
    // init arm positions using default offset
    public void initArm(){
        // set pidf coefficients
        PIDFCoefficients pidf = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine(pidf.toString());
        telemetry.update();
        
        PIDFCoefficients newPidf = new PIDFCoefficients(15, 0.049988, 0.01, 0);
        
        try {
            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPidf);
        } catch( java.lang.UnsupportedOperationException e ){
            telemetry.addLine(e.toString());
            telemetry.update();
        }
        
        for(int i = 0; i < armPositions.length; i++){
            armPositions[i] = armPositions[i] + armOffset;
        }
    }
    
    // init arm using custom offset override
    public void initArm(int offset){
        for(int i = 0; i < armPositions.length; i++){
            armPositions[i] = armPositions[i] + offset;
        }
    }
    
    // init arm using custom offset
    // if addToOffset is true, it adds to the offset instead of overriding it
    public void initArm(int offset, boolean addToOffset){
        if(addToOffset){
            for(int i = 0; i < armPositions.length; i++){
                armPositions[i] = armPositions[i] + armOffset + offset;
            }
        } else {
            for(int i = 0; i < armPositions.length; i++){
                armPositions[i] = armPositions[i] + offset;
            }
        }
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
    public void initVuforia() {
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
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
    public void unclamp(){
        clawServoLeft.setPosition(0.0);
        clawServoRight.setPosition(0.83);
    }
    
    public void clamp(){
        clawServoLeft.setPosition(0.435);
        clawServoRight.setPosition(0.365);
    }
    
    // move forward at a speed of inches / second
    public void goForwardSpeed(double velocity){
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        double revolutions = velocity / this.wheelCircumference;
        double ticks = revolutions * this.tpr;
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        this.backLeft.setVelocity(ticks);
        this.backRight.setVelocity(ticks);
        this.frontLeft.setVelocity(ticks);
        this.frontRight.setVelocity(ticks);
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
        
        this.intendedRotation = finalOrientation + this.startingOrientation;
        
        double extraSpeed = 0.2;
        
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        boolean flipped = false;
        double lastOrientation = this.getYRotation();
        
        if(finalOrientation < startingOrientation){
            this.backLeft.setPower(speed);
            this.backRight.setPower(-speed);
            this.frontLeft.setPower(speed);
            this.frontRight.setPower(-speed);
            
            while(this.getYRotation() > finalOrientation && !flipped && opModeIsActive()){
                double speedFactor = 1 - Math.abs( (this.getYRotation()-startingOrientation) / finalOrientation );
                
                // cache so that value doesn't change between checks
                double rot = this.getYRotation();
                
                // check for flip
                if(rot - lastOrientation > 300 || rot - lastOrientation < -300){
                    flipped = true;
                    break;
                }
                
                lastOrientation = rot;
                
                // small factor to prevent robot from moving slow
                speedFactor += extraSpeed;
                
                this.backLeft.setPower(speed * speedFactor);
                this.backRight.setPower(-speed * speedFactor);
                this.frontLeft.setPower(speed * speedFactor);
                this.frontRight.setPower(-speed * speedFactor);
                
                
                telemetry.addData("this.getYRotation() - lastOrientation", this.getYRotation() - lastOrientation);
                /*telemetry.addData("starting orientation", startingOrientation);
                telemetry.addData("final orientation", finalOrientation);
                telemetry.addData("current gyro", this.getYRotation());
                telemetry.addData("speedFactor", speedFactor);*/
                telemetry.update();
            }
        } else {
            this.backLeft.setPower(-speed);
            this.backRight.setPower(speed);
            this.frontLeft.setPower(-speed);
            this.frontRight.setPower(speed);
            
            while(this.getYRotation() < finalOrientation && !flipped && opModeIsActive()){
                double speedFactor = 1 - Math.abs( (this.getYRotation()-startingOrientation) / finalOrientation );
                
                // cache so that value doesn't change between checks
                double rot = this.getYRotation();
                
                // check for flip
                if(rot - lastOrientation > 300 || rot - lastOrientation < -300){
                    flipped = true;
                    break;
                }
                
                lastOrientation = rot;
                
                // small factor to prevent robot from moving slow
                speedFactor += extraSpeed;
                
                this.backLeft.setPower(-speed * speedFactor);
                this.backRight.setPower(speed * speedFactor);
                this.frontLeft.setPower(-speed * speedFactor);
                this.frontRight.setPower(speed * speedFactor);
                
                
                telemetry.addData("this.getYRotation() - lastOrientation", this.getYRotation() - lastOrientation);
                /*telemetry.addData("starting orientation", startingOrientation);
                telemetry.addData("final orientation", finalOrientation);
                telemetry.addData("current gyro", this.getYRotation());
                telemetry.addData("speedFactor", speedFactor);*/
                telemetry.update();
            }
        }
        
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
    }
    
    // pivot to a specific orientation
    public void pivotToAngle(double orientation, double speed){
        double startingOrientation = this.getYRotation();
        double finalOrientation = orientation+this.startingOrientation;
        
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
                double speedFactor = 1 - (this.getYRotation()-startingOrientation) / (finalOrientation - startingOrientation);
                
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
                double speedFactor = 1 - (this.getYRotation()-startingOrientation) / (finalOrientation - startingOrientation);
                
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
    
    public void wait(double time){
        double startTime = this.globalTime.seconds();
        
        while(this.globalTime.seconds() < startTime + time && opModeIsActive());
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
    
    // get the x coordinate of pipe recognition
    public float getPipeRecognition(){
        // get pipe recognition x coordinates if there are any (or negative if not)
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            
            if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
                Recognition recognition;
                
                float width;
                
                int i = 0;
                do {
                    recognition = updatedRecognitions.get(i);
                    width = recognition.getRight() - recognition.getLeft();
                    i++;
                } while(i < updatedRecognitions.size() && width > 250 && opModeIsActive());
                
                if(width > 250){
                    return -1;
                }
                
                return recognition.getLeft();
            }
        }
        
        return -1;
    }
    
    // wait for a pipe recognition until max time or detected
    public float waitForPipeRecognition(double time){
        double startTime = globalTime.seconds();
        
        boolean detected = false;
        float x = -1;
        
        while(x < 0 && globalTime.seconds() < startTime+time && opModeIsActive()){
            x = getPipeRecognition();
        }
        
        return x;
    }
    
    // get the average color of the camera and test if it is yellow
    public boolean waitForDuck(double r, double g, double b, int minimumFrequency, double colorThreshold) throws java.lang.InterruptedException {
        double startTime = globalTime.seconds();
        
        boolean detected = false;
        
        // frequency
        double red = 0;
        double green = 0;
        double blue = 0;
        
        double rtog = g == 0 ? 0 : r / g;
        double rtob = b == 0 ? 0 : r / b;
        double gtor = r == 0 ? 0 : g / r;
        double gtob = b == 0 ? 0 : g / b;
        double btor = r == 0 ? 0 : b / r;
        double btog = g == 0 ? 0 : b / g;
        
        telemetry.addData("rtog", rtog);
        telemetry.addData("rtob", rtob);
        telemetry.addData("gtob", gtob);
        telemetry.update();
        
        int frequency = 0;
        
        Frame frame = vuforia.getFrameQueue().take();
        android.graphics.Bitmap bmp = vuforia.convertFrameToBitmap(frame);
        int[] pixels = new int[bmp.getWidth()*bmp.getHeight()];
        
        try {
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());
        } catch( Exception e ){
            telemetry.addLine("Problem fetching bmp pixels");
            telemetry.update();
            return false;
        }
        
        // get average of pixels
        for(int i = 0; i < pixels.length; i++){
            red = android.graphics.Color.red(pixels[i]);
            green = android.graphics.Color.green(pixels[i]);
            blue = android.graphics.Color.blue(pixels[i]);
            
            //if(red > r-colorThreshold && red < r+colorThreshold && green > g-colorThreshold && green < g+colorThreshold && blue > b-colorThreshold && blue < b+colorThreshold){
            if(
                /*red > green*rtog - colorThreshold && red < green*rtog + colorThreshold && 
                red > blue*rtob - colorThreshold && red < blue*rtob + colorThreshold && 
                green > blue*gtob - colorThreshold && green < blue*gtob + colorThreshold &&
                green > red*gtor - colorThreshold && green < red*gtor + colorThreshold &&
                blue > red*btor - colorThreshold && blue < red*btor + colorThreshold &&
                blue > green*btog - colorThreshold && blue < green*btog + colorThreshold*/
                approximatelyEqual(green == 0 ? 0 : red/green, rtog, colorThreshold) &&
                approximatelyEqual(blue == 0 ? 0 : red/blue, rtob, colorThreshold) &&
                approximatelyEqual(red == 0 ? 0 : green/red, gtor, colorThreshold) &&
                approximatelyEqual(blue == 0 ? 0 : green/blue, gtob, colorThreshold) &&
                approximatelyEqual(red == 0 ? 0 : blue/red, btor, colorThreshold) &&
                approximatelyEqual(green == 0 ? 0 : blue/green, btog, colorThreshold)
            ){
                if(i % 5000 == 0){
                    telemetry.addData("i", i);
                    telemetry.addData("r", red);
                    telemetry.addData("g", green);
                    telemetry.addData("b", blue);    
                }
                
                frequency++;
            }
        }
        
        telemetry.addData("frequency", frequency);
        //telemetry.update();
        
        telemetry.addData("Detection?: ", frequency >= minimumFrequency);
        telemetry.update();
        
        while(opModeIsActive());
        
        return (frequency >= minimumFrequency);
    }
    
    // whether two values are approximately equal within a threshold range
    public boolean approximatelyEqual(double val1, double val2, double threshold){
        return (val1 > val2-threshold && val1 < val2+threshold && val2 > val1-threshold && val2 < val1+threshold);
    }
    
    // look for a duck in the camera until one is found, or until time is > maxTime 
    public boolean waitForDuck(double maxTime){
        // wait for detection (or for 2.5 seconds)
        double startTime = globalTime.seconds();
        
        boolean detected = false;
        
        while(globalTime.seconds() < startTime + maxTime && !detected && opModeIsActive()){
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
    
    // move backwards until the carousel velocity slows down (assumes the resistance is the motor touching the carousel)
    // returns the distance it travelled backwards
    public double moveBackUntilCarousel(){
        double startTime = this.globalTime.seconds();
        
        // turn on carousel
        double expectedVelocity = 180;
        double velocityThreshold = 12; // allowed deviation from angular velocity before it assumes it's reached the carousel
        
        this.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        this.carousel.setVelocity(expectedVelocity, AngleUnit.DEGREES);
        
        // start moving backwards
        double moveSpeed = -5.5; // inches / second
        
        double moveStart = this.globalTime.seconds();
        
        this.goForwardSpeed(moveSpeed);
        
        // this doesn't need to be assigned a value, but it can't hurt just in case
        double carouselVelocity = expectedVelocity;
        
        // wait a sec for motor to spin up
        startTime = this.globalTime.seconds();
        
        while(this.globalTime.seconds() < startTime+1 && opModeIsActive());
        
        do {
            carouselVelocity = this.carousel.getVelocity(AngleUnit.DEGREES);
            
            //telemetry.addData("v", carouselVelocity);
            //telemetry.update();
        } while(approximatelyEqual(expectedVelocity, carouselVelocity, velocityThreshold) && opModeIsActive());
        
        this.backLeft.setVelocity(0);
        this.backRight.setVelocity(0);
        this.frontLeft.setVelocity(0);
        this.frontRight.setVelocity(0);
        
        double elapsedTime = this.globalTime.seconds() - moveStart;
        double distanceTravelled = moveSpeed * elapsedTime;
        
        // run carousel for 2.5 seconds
        startTime = this.globalTime.seconds();
        double runTime = 2.5;
        
        while(this.globalTime.seconds() < startTime+runTime && opModeIsActive());
        
        return distanceTravelled;
    }
    
    public double toRadians(double degrees){
        return degrees * Math.PI / 180;
    }
    
    public double toDegrees(double radians){
        return radians * 180 / Math.PI;
    }
    
    public void setArmLevel(int level, int offset){
        armMotor.setTargetPosition(this.armPositions[level] + offset);
        
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }
}