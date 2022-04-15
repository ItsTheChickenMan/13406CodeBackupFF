package AutoTools;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Frame;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class EzrasLaw extends LinearOpMode {
  // TODO: document attributes
  public Blinker control_Hub;
  public Blinker expansion_Hub_2;
  
  public HardwareDevice webcam_1;
  
  public DcMotorEx armMotor;

  public DcMotorEx backLeft;
  public DcMotorEx backRight;
  public DcMotorEx frontLeft;
  public DcMotorEx frontRight;

  public DcMotorEx carousel;
  
  public Servo clawServoLeft;
  public Servo clawServoRight;
  
  private BNO055IMU imu;
  
  public CRServo tapeBlue;
  public CRServo tapeRed;
  
  public static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/FreightFrenzy_PIPE.tflite";
  
  public static final String[] LABELS = new String[] { "Pipe" };
  
  public static final String VUFORIA_KEY = "AcnDHVv/////AAABmRnrh4YWCUq9oXJiDJfQ4tpGcyqNmoiVhQEAVeBCc1pxc59UM914g/AfIlzXvB+DEj61Op1AD7CdC/shuEqyF7E9jn4nYrzRhWXBB2ZZOISZ7dsulJ3ZxGsbSF6xpAuvnGoVTZL2wZGVzFvHmz7re2Ek7IH7Rc59kzFI2mkvtUxHPU4hqR14fXBNvHG5bdVoidyZVm50h5nyDdbnOij/2sJXYh01NzH02y2O1voPzA2XT6sEKAgqi9j8UQ6q4XP/wSa1BLPwwcInLnEJohQSOED4GGWW+M9WHZ6t2zUkSge+YPzBINQwczICkTG6qfDwlRKb7ghKYjvScxr2quuUnIGZntALNrHSlP5OQlBW7QHf";
  
  public VuforiaLocalizer vuforia;
  
  public TFObjectDetector tfod;
  
  public ElapsedTime globalTime;
  
  public double gearRatio = 30.0/14.0;
  public double tpr = 384.5 * this.gearRatio;
  public double wheelDiameter = 4.0;
  public double wheelCircumference = this.wheelDiameter * Math.PI;
  public double strafeMultiplier = 1.045;
  
  public double autoSpeed = 40.0D;
  
  public double blockingPauseTime = 0.0;
  
  public double startingOrientation = 0.0D;
  public double intendedRotation = 0.0D;
  
  public double x;
  public double y;
  
  public String[] finalMessages = new String[] { "mission accomplished", "payload delivered", "job well done", "pheonix is a genius", "tyler is better than saeid", "the cake is a lie", "the Baked Ziti King reigns supreme"};
  
  public int[] armPositions = new int[] {-975, -918, -815, -714, -550};
  
  public int armOffset = 0;
  
  /**
    @brief Initialize all parts to their values
  */
  public void initVals() {
    this.control_Hub = (Blinker)this.hardwareMap.get(Blinker.class, "Control Hub");
    this.expansion_Hub_2 = (Blinker)this.hardwareMap.get(Blinker.class, "Expansion Hub 2");
    this.armMotor = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "armMotor");
    this.backLeft = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "backLeft");
    this.backRight = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "backRight");
    this.carousel = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "carousel");
    this.frontLeft = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "frontLeft");
    this.frontRight = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "frontRight");
    this.tapeBlue = (CRServo)this.hardwareMap.get(CRServo.class, "tapeBlue");
    this.tapeRed = (CRServo)this.hardwareMap.get(CRServo.class, "tapeRed");
    this.clawServoLeft = (Servo)this.hardwareMap.get(Servo.class, "clawServoLeft");
    this.clawServoRight = (Servo)this.hardwareMap.get(Servo.class, "clawServoRight");
    this.imu = (BNO055IMU)this.hardwareMap.get(BNO055IMU.class, "imu");
  }
  
  /**
    * @brief Initialize arm PIDF and offset
    *
    * Initialize the arm's PIDF coefficients and apply the arm offset to the global arm positions
  */
  public void initArm() {
    this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    PIDFCoefficients pidf = this.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    this.telemetry.addLine(pidf.toString());
    this.telemetry.update();
    PIDFCoefficients newPidf = new PIDFCoefficients(5.0, pidf.i, pidf.d, pidf.f, MotorControlAlgorithm.LegacyPID);
    try {
      this.armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPidf);
    } catch (UnsupportedOperationException e) {
      this.telemetry.addLine(e.toString());
      this.telemetry.update();
      
      while(opModeIsActive());
    } 
    for (int i = 0; i < this.armPositions.length; i++)
      this.armPositions[i] = this.armPositions[i] + this.armOffset; 
  }
  
  /**
    * @brief Set each motor's mode to one mode
  */
  public void setMotorMode(DcMotor.RunMode mode){
    this.backLeft.setMode(mode);
    this.backRight.setMode(mode);
    this.frontLeft.setMode(mode);
    this.frontRight.setMode(mode);
  }

  /**
    * @brief Set each motor's power to one value
  */
  public void setPower(double power) {
    this.frontLeft.setPower(power);
    this.frontRight.setPower(power);
    this.backLeft.setPower(power);
    this.backRight.setPower(power);
  }
  
  /**
    * @brief Set each motor's power to an individual value
  */
  public void setPowers(double fl, double fr, double bl, double br) {
    this.frontLeft.setPower(fl);
    this.frontRight.setPower(fr);
    this.backLeft.setPower(bl);
    this.backRight.setPower(br);
  }
  
  /**
    * @brief Set each motor's velocity to one value
    *
    * @note does not set motor's mode to any particular value
    *
    * @param velocity velocity, in ticks per second
  */
  public void setVelocity(double velocity){
    this.frontLeft.setVelocity(velocity);
    this.frontRight.setVelocity(velocity);
    this.backLeft.setVelocity(velocity);
    this.backRight.setVelocity(velocity);
  }
  
  /**
    * @brief Set each motor's velocity to one value
    *
    * @note does not set motor's mode to any particular value
    *
    * @param fl frontLeft's velocity, in ticks per second
    * @param fr frontRight's velocity, in ticks per second
    * @param bl backLeft's velocity, in ticks per second
    * @param br backRight's velocity, in ticks per second
  */
  public void setVelocity(double fl, double fr, double bl, double br){
    this.frontLeft.setVelocity(fl);
    this.frontRight.setVelocity(fr);
    this.backLeft.setVelocity(bl);
    this.backRight.setVelocity(br);
  }

  /**
    * @brief Set each motor's position to one value
    *
    * @note does not set motor's mode to any particular value
    * @note position is a double, despite needing to be casted to an integer, because in most calculations ticks is a double
    * 
    * @param position position, in ticks
  */
  public void setPosition(double position){
    this.backLeft.setTargetPosition((int)position);
    this.backRight.setTargetPosition((int)position);
    this.frontLeft.setTargetPosition((int)position);
    this.frontRight.setTargetPosition((int)position);
  }
  
  /**
    * @brief Set each motor's position to one value
    *
    * @note does not set motor's mode to any particular value
    * @note each parameter is a double, despite needing to be casted to an integer, because in most calculations ticks is a double

    * @param fl frontLeft's position, in ticks
    * @param fr frontRight's position, in ticks
    * @param bl backLeft's position, in ticks
    * @param br backRight's position, in ticks
  */
  public void setPosition(double fl, double fr, double bl, double br){
    this.backLeft.setTargetPosition((int)fl);
    this.backRight.setTargetPosition((int)fr);
    this.frontLeft.setTargetPosition((int)bl);
    this.frontRight.setTargetPosition((int)br);
  }

  /**
    * @brief Go forward at a certain power
    *
    * @note power, not velocity
  */
  public void forward(double power) {
    this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    this.setPower(power);
  }
  
  /**
    * @brief Strafe at a certain power
    *
    * @note power, not velocity
  */
  public void strafe(double power) {
    this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    this.setPowers(power, -power, -power, power);
  }
  
  /**
    * @brief Turn/pivot at a certain power
    *
    * @note power, not velocity
  */
  public void turn(double power) {
    this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    this.setPowers(power, -power, power, -power);
  }
  
  /**
    * @brief Stop all motors (set power to 0)
  */
  public void stopMotors() {
    this.setPower(0.0);
  }
  
  /**
    * @brief Initialize an instance of vuforia
    *
    * Initialize an instance of the VuforiaLocalizer class, using the global VUFORIA_KEY as a license key and "Webcam 1" (in config) as the camera.
    *
    * @pre this.VUFORIA_KEY is set to a valid license key
    * @post initalizes global this.vuforia
  */
  public void initVuforia() {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.vuforiaLicenseKey = this.VUFORIA_KEY;
    parameters.cameraName = (CameraName)this.hardwareMap.get(WebcamName.class, "Webcam 1");
    this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }
  
  /**
    * @brief Initialize an instance of TFObjectDetector
    *
    * @pre this.vuforia must be a valid instance of VuforiaLocalizer, not null (call initVuforia before this function)
    * this.TFOD_MODEL_ASSET is a path to a .tflite model
    *
    * @post initializes global this.tfod
  */
  public void initTfod() {
    int tfodMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", this.hardwareMap.appContext
        .getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.8F;
    tfodParameters.isModelTensorFlow2 = true;
    tfodParameters.inputSize = 320;
    this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
    this.tfod.loadModelFromFile(this.TFOD_MODEL_ASSET, this.LABELS);
  }
  
  /**
    * @brief Quickly deposit clamped freight
    *
    * Quickly deposit clamped freight by slightly opening claw, waiting for 0.2 seconds, and clamping again.
    *
    * The primary use of this function is when depositing to the shared hub.  If the normal unclamp function is used, the claw could potentially "hook" the pole of the shared hub, causing penalties and delays.  This avoids that by only opening slightly, and quickly closing after.
  */
  public void quickDeposit(){
    this.clawServoLeft.setPosition(0.26D);  
    this.clawServoRight.setPosition(0.56D);
    
    wait(0.2);
    
    this.clamp();
  }
  
  /**
    * @brief Fully unclamp the claw servos to release freight
    *
    * @todo Store claw positions as a global attribute
    * @todo Store claw state (clamped/unclamped) as a global attribute
  */
  public void unclamp() {
    this.clawServoLeft.setPosition(0.0D);
    this.clawServoRight.setPosition(0.83D);
  }
  
  /**
    * @brief Fully clamp the claw servos to grab freight
    *
    * @todo Same todos as this.unclamp()
  */
  public void clamp() {
    this.clawServoLeft.setPosition(0.435D);
    this.clawServoRight.setPosition(0.365D);
  }
  
  /**
    * @brief Go forward at a velocity in in/s
    *
    * @param velocity velocity to travel in inches/second
  */
  public void goForwardSpeed(double velocity) {
    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // calculate ticks per second 
    double ticks = this.velocityToTicks(velocity);

    // set run mode to run w/ encoders
    this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODERS);

    // set velocity to tps
    this.setVelocity(ticks);
  }
  
  /**
    * @brief Strafe at a velocity in in/s
    *
    * @param velocity velocity to travel in inches/second
    *
    * @todo Does not account for strafe multiplier and will likely be innacurate, especially at high speeds
  */
  public void strafeSpeed(double velocity){
    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // calculate ticks per second    
    double ticks = velocityToTicks(velocity);
    
    // set motors to run w/ encoders
    this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    // set tpr
    this.setVelocity(ticks, -ticks, -ticks, ticks);
  }
  
  /**
    * @brief Convert ticks/s to in/s
    *
    * @param ticks tpr to convert to in/s
    * 
    * @return velocity in in/s
    *
    * @todo Change name to something more general
  */
  public double ticksToVelocity(double ticks){
    double revolutions = ticks / this.tpr;
    double velocity = revolutions * this.wheelCircumference;
    
    return velocity;
  }
  
  /**
    * @brief Convert in/s to ticks/s
    *
    * @param velocity in/s to convert to tpr
    *
    * @return velocity in tpr
    *
    * @todo Change name to something more general
  */
  public double velocityToTicks(double velocity){
    double revolutions = velocity / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    
    return ticks;
  }
  
  /**
    * @brief Go forward a specific distance at a specific velocity in inches per second
    *
    * @param distance distance, in inches, to travel
    * @param speed velocity, in inches per second, to travel
  */
  public void goForwardDistance(double distance, double speed) {
    // multiply speed by relative speed of auto
    speed *= this.autoSpeed;

    // convert tick to travel to
    double ticks = velocityToTicks(distance);
    
    // convert speed to speed in ticks
    speed = velocityToTicks(speed);
    
    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // set motor positions
    this.setPosition(ticks);
    
    // set modes to run to assigned position
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // set motor velocity
    this.setVelocity(speed);

    // TODO: leaving this here for fun, but this needs to be redone or deleted since it's too broken and incomplete to be useful at the moment
    this.x += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.cos(toRadians(-this.intendedRotation)) * distance;
  }
  
  /**
    * @brief Go forward a specific distance at a specific velocity in inches per second
    *
    * @param distance distance, in inches, to travel
    * @param speed velocity, in inches per second, to travel
    * @param blocking if function should block execution until action is complete
  */
  public void goForwardDistance(double distance, double speed, boolean blocking) {
    // multiply speed by relative speed of auto
    speed *= this.autoSpeed;
    
    // convert tick to travel to
    double ticks = velocityToTicks(distance);
    
    // convert speed to speed in ticks
    speed = velocityToTicks(speed);
    
    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // set motor positions
    this.setPosition(ticks);
    
    // set modes to run to assigned position
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // set motor velocity
    this.setVelocity(speed);

    // block thread execution until motors have reached intended destination
    // TODO: probably can't do before comp, but come up with some kind of simple multi-thread model (maybe utlize iterative opmode?) to allow for multiple execution of multiple tasks w/o having to block thread execution
    if (blocking){
      // wait for motors to stop
      while (opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy());
      wait(blockingPauseTime);
    }

    // TODO: leaving this here for fun, but this needs to be redone or deleted since it's too broken and incomplete to be useful at the moment
    this.x += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.cos(toRadians(-this.intendedRotation)) * distance;
  }
  
  /**
    * @brief Strafe a specific distance at a specific velocity in inches per second
    *
    * @param distance distance, in inches, to travel
    * @param speed velocity, in inches per second, to travel
  */
  public void strafeDistance(double distance, double speed) {
    // multiply speed by relative speed of auto
    speed *= this.autoSpeed;

    // convert tick to travel to
    double ticks = velocityToTicks(distance);
    
    // convert speed to speed in ticks
    speed = velocityToTicks(speed);
    
    // multiply ticks by strafe multiplier
    // TODO: more accurate strafing, probably easiest just to throw odometry wheels on
    ticks *= this.strafeMultiplier;

    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // set motor positions
    this.setPosition(ticks, -ticks, -ticks, ticks);
    
    // set modes to run to assigned position
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // set motor velocity
    this.setVelocity(speed);

    // TODO: leaving this here for fun, but this needs to be redone or deleted since it's too broken and incomplete to be useful at the moment
    this.x += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.cos(toRadians(-this.intendedRotation)) * distance;
  }
  
  /**
    * @brief Strafe a specific distance at a specific velocity in inches per second
    *
    * @param distance distance, in inches, to travel
    * @param speed velocity, in inches per second, to travel
    * @param blocking if function should block execution until action is complete
  */
  public void strafeDistance(double distance, double speed, boolean blocking) {
    // multiply speed by relative speed of auto
    speed *= this.autoSpeed;
    
    distance = -distance; // sshhh TODO: replace
    
    // convert tick to travel to
    double ticks = velocityToTicks(distance);
    
    // multiply ticks by strafe multiplier
    // TODO: more accurate strafing, probably easiest just to throw odometry wheels on
    ticks *= this.strafeMultiplier;
    
    // convert speed to speed in ticks
    speed = velocityToTicks(speed);

    // reset drive motors
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // set motor positions
    this.setPosition(ticks, -ticks, -ticks, ticks);
    
    // set modes to run to assigned position
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // set motor velocity
    this.setVelocity(speed);

    // block thread execution until motors have reached intended destination
    // TODO: probably can't do before comp, but come up with some kind of simple multi-thread model (maybe utlize iterative opmode?) to allow for multiple execution of multiple tasks w/o having to block thread execution
    if (blocking){
      // wait for motors to stop
      while (opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy());
      wait(blockingPauseTime);
    }
    
    // TODO: see related todos
    this.x += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.cos(toRadians(-this.intendedRotation)) * distance;
  }
  
  /**
    * @brief Move forward and strafe a certain distance at the same time
    * 
    * @param forward inches to move forward
    * @param strafe inches to strafe
    * @param velocity velocity to move at
  */
  public void forwardStrafeDistance(double forward, double strafe, double velocity){
    // reset encoders
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // calculate net movement for each motor
    double forwardTicks = velocityToTicks(forward);
    double strafeTicks = velocityToTicks(strafe);
    
    strafeTicks *= this.strafeMultiplier;
    
    // get velocity in ticks
    velocity = velocityToTicks(velocity);
    
    telemetry.addData("forward", forwardTicks);
    telemetry.addData("strafe", strafeTicks);
    telemetry.addData("velocity", velocity);
    telemetry.update();
    
    double fl = forwardTicks - strafeTicks;
    double fr = forwardTicks + strafeTicks;
    double bl = forwardTicks + strafeTicks;
    double br = forwardTicks - strafeTicks;
    
    // set motor positions
    this.setPosition(fl, fr, bl, br);
    
    // set mode to run to pos
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // set velocity
    this.setVelocity(velocity);
  }
  
  /**
    * @brief Pivot by a specific angle in degrees, at a certain power
    *
    * @todo Completely rewrite so that it doesn't setup imu each time, handles pivoting speed, much more effectively, and uses setVelocity instead of setPower
    *
    * @param angle angle to pivot by
    * @speed power to motors as a percent from 0-1 
  */
  public void pivotAngle(double angle, double speed) {
    // reset imu to prevent flip-flop of gyro unless past 180
    setupIMU();
    
    // get current rotation
    // TODO: i'm not rewriting things at the moment, but this should always be 0 so why is the fetched?
    // I must've been high of half asleep or something when I wrote this
    double startingOrientation = getYRotation();

    // get final orientation of bot after action
    double finalOrientation = startingOrientation + angle;
    
    // check for flip-flop
    if (finalOrientation > 180.0D || finalOrientation < -180.0D)
      finalOrientation = 360.0D - finalOrientation;

    // store intended rotation of the bot (for x, y coordinate tracking, which we don't use anymore)
    this.intendedRotation = finalOrientation + this.startingOrientation;

    // additional power to give motors instead of just linear power
    double extraSpeed = 0.15D;
    
    // set motors to run w/o encoders
    this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // if orientation has flipped yet
    boolean flipped = false;
    
    // last orientation of the bot for checking flipping
    double lastOrientation = getYRotation();
    
    // check which direction we need to rotate
    if (finalOrientation < startingOrientation) {
      // run while gyro hasn't reached finalOrientation AND gyro hasn't flipped
      while (getYRotation() > finalOrientation && !flipped && opModeIsActive()) {
        // get speed factor
        // TODO: don't use linear power throughout the entire action, but rather a small portion (maybe 1/4) of the action so that it's smooth but fast 
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);

        // TODO: why is this fetched here and not before speedFactor calculations???
        double rot = getYRotation();

        // check for flip
        // TODO: why does flipped exist when it calls break anyways???
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        
        // set lastOrientation to current orientation
        lastOrientation = rot;

        // add extra speed
        speedFactor += extraSpeed;
        
        // TODO: I'm too lazy to replace this with the proper function call, it's all getting replaced anyways
        this.backLeft.setPower(-speed * speedFactor);
        this.backRight.setPower(speed * speedFactor);
        this.frontLeft.setPower(-speed * speedFactor);
        this.frontRight.setPower(speed * speedFactor);
      } 
    } else {
      // run while gyro hasn't reached finalOrientation AND gyro hasn't flipped
      while (getYRotation() < finalOrientation && !flipped && opModeIsActive()) {
        // get speed factor
        // TODO: same as before
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);

        // TODO: same as before
        double rot = getYRotation();

        // check for flip
        // TODO; same as before
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 

        // set lastOrientation to current orientation
        lastOrientation = rot;
        
        // add extra speed
        speedFactor += extraSpeed;
        
        // ditto
        // TODO: the only difference between both loops is the sign it checks for in orientation and the direction it rotates, why couldn't have this just been stored as a direction (1, -1)???
        this.backLeft.setPower(speed * speedFactor);
        this.backRight.setPower(-speed * speedFactor);
        this.frontLeft.setPower(speed * speedFactor);
        this.frontRight.setPower(-speed * speedFactor);
      } 
    } 

    // stop motors
    this.backLeft.setPower(0.0D);
    this.backRight.setPower(0.0D);
    this.frontLeft.setPower(0.0D);
    this.frontRight.setPower(0.0D);
  }
  
  /**
    * @brief Pivot to a specific angle, in degrees
    *
    * @note this is broken, don't use it.  I only leave it in here so that we know we need a version that works.
    *
    * @todo write a version of this that works
    *
    * @param who cares
  */
  public void pivotToAngle(double angle, double speed) {
    // TODO: I'm not documenting this because we'll be deleting it shortly.
    double startingOrientation = getYRotation();
    double finalOrientation = angle;
    if (finalOrientation > 180.0D || finalOrientation < -180.0D)
      finalOrientation = 360.0D - finalOrientation; 
    this.intendedRotation = finalOrientation + this.startingOrientation;
    double extraSpeed = 4.0D;
    this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    boolean flipped = false;
    double lastOrientation = getYRotation();
    if (finalOrientation < startingOrientation) {
      /*this.backLeft.setPower(speed);
      this.backRight.setPower(-speed);
      this.frontLeft.setPower(speed);
      this.frontRight.setPower(-speed);*/
      while (getYRotation() > finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setVelocity(-speed * speedFactor);
        this.backRight.setVelocity(speed * speedFactor);
        this.frontLeft.setVelocity(-speed * speedFactor);
        this.frontRight.setVelocity(speed * speedFactor);
        //this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        //this.telemetry.update();
      } 
    } else {
      /*this.backLeft.setPower(-speed);
      this.backRight.setPower(speed);
      this.frontLeft.setPower(-speed);
      this.frontRight.setPower(speed);*/
      while (getYRotation() < finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setVelocity(speed * speedFactor);
        this.backRight.setVelocity(-speed * speedFactor);
        this.frontLeft.setVelocity(speed * speedFactor);
        this.frontRight.setVelocity(-speed * speedFactor);
        telemetry.addData("this.getYRotation()", this.getYRotation());
        telemetry.addData("finalOrientation", finalOrientation);
        telemetry.addData("getYRotation() >= finalOrientation ? ", getYRotation() >= finalOrientation);
        this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        this.telemetry.update();
      } 
    } 
    this.backLeft.setPower(0.0D);
    this.backRight.setPower(0.0D);
    this.frontLeft.setPower(0.0D);
    this.frontRight.setPower(0.0D);
  }
  
  /**
    * @brief Setup IMU for reading gyroscope values
    *
    * @pre imu is initialized to an instance of BNO055IMU (can be done by calling initVals)
  */
  public void setupIMU() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    this.imu.initialize(parameters);
  }
  
  /**
    * @brief Get the current y rotation of the bot from the imu
    * 
    * @pre this.imu is a valid instance of the BNO055IMU class and is initialized using setupIMU
  */
  public double getYRotation() {
    return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle;
  }
  
  /**
    @brief Returns the left boundary of the LAST recognition of the pipe it sees, if any
    *
    * Returns the left boundary of the last pipe recognition it sees, or -1, provided the width of the recognition is not greater than 250 pixels
    *
    * @pre this.tfod is initialized
    *
    * @return left boundary of the last pipe detection, or -1 if none were found
  */
  public float getPipeRecognition() {
    // check if tensorflow is initialized
    if (this.tfod != null) {
      // get updated recognitions
      List<Recognition> updatedRecognitions = this.tfod.getUpdatedRecognitions();
      
      // NOTE: updatedRecognitions != null should be enough, but sometimes it will be not be null, and still 0
      if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
        // loop through recognitions

        // recognition
        Recognition recognition;

        // width of recognition
        float width;
        
        // loop
        int i = 0; // counter
        do {
          // get recognition
          recognition = updatedRecognitions.get(i);

          // width of recognition
          width = recognition.getRight() - recognition.getLeft();

          // increment counter
          ++i;
        } while (i < updatedRecognitions.size() && width > 250.0F && opModeIsActive()); // run until all detections are cycled through
        // TODO: seems to be a weird logic error, why cycle until last detection?

        // TODO: another strange thing, I'm too tired to look at it close
        if (width > 250.0F)
          return -1.0F;

        return recognition.getLeft();
      } 
    }

    // failed, return -1
    return -1.0F;
  }
  
  /**
    * @brief Wait for a specified amount of time for a pipe recognition, and return either the left boundary if found or -1 if not
    *
    * @param time time to wait for detection, in seconds
    * @return left boundary of detection
  */
  public float waitForPipeRecognition(double time) {
    // get start time
    double startTime = this.globalTime.seconds();
    
    // is detected?
    boolean detected = false;

    // left bound
    float x = -1.0F;

    // run while there's time and no detection yet
    while (x < 0.0F && this.globalTime.seconds() < startTime + time && opModeIsActive())
      x = getPipeRecognition(); 

    // return left bound
    return x;
  }
  
  /**
    * @brief Check if two double values are approximately equal to each other within a range threshold
    *
    * @param val1 first val to compare
    * @param val2 second val to compare
    * @param threshold maximum range which values are allowed to deviate from each other before being considered unequal
    * @return true if approx. equal, or false if not
  */
  public boolean approximatelyEqual(double val1, double val2, double threshold) {
    return (val1 > val2 - threshold && val1 < val2 + threshold && val2 > val1 - threshold && val2 < val1 + threshold);
  }
  
  /**
    * @brief Move backward until carousel is hit
    *
    * @param waitTime time, in seconds, to wait at carousel for duck
    * @return distance travelled while moving back, in inches
    *
    * @TODO document
  */
  public double moveBackUntilCarousel(double waitTime) {
    double startTime = this.globalTime.seconds();
    double expectedVelocity = 175.0D;
    double velocityThreshold = 12.0D;
    this.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.carousel.setVelocity(expectedVelocity, AngleUnit.DEGREES);
    double moveSpeed = -5.5D;
    double moveStart = this.globalTime.seconds();
    goForwardSpeed(moveSpeed);
    double carouselVelocity = expectedVelocity;
    startTime = this.globalTime.seconds();
    
    double distanceTravelled = 0;
    double lastTime = startTime;
    
    while (this.globalTime.seconds() < startTime + 1.0D&& opModeIsActive());
    do {
      carouselVelocity = this.carousel.getVelocity(AngleUnit.DEGREES);
      
      double vel = ticksToVelocity(this.backLeft.getVelocity());
      
      double elapsed =this.globalTime.seconds();
      
      double dif = elapsed - lastTime;
      lastTime = elapsed;
      
      distanceTravelled += vel * dif;
    } while (approximatelyEqual(expectedVelocity, carouselVelocity, velocityThreshold) && opModeIsActive());
    double finished = this.globalTime.seconds();
    
    this.backLeft.setVelocity(0.0D);
    this.backRight.setVelocity(0.0D);
    this.frontLeft.setVelocity(0.0D);
    this.frontRight.setVelocity(0.0D);
    
    startTime = finished;
    double runTime = waitTime;
    while (this.globalTime.seconds() < startTime + runTime && opModeIsActive());
    return -distanceTravelled;
  }
  
  /**
    * @brief Strafe sideways until carousel is hit
    *
    * @param waitTime time, in seconds, to wait at carousel for duck
    * @return distance travelled while moving, in inches
    *
    * @TODO document
  */
  public double strafeBackUntilCarousel(double waitTime){
    double startTime = this.globalTime.seconds();
    
    double expectedVelocity = -165.0D;
    double velocityThreshold = 12.0D;
    
    this.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.carousel.setVelocity(expectedVelocity, AngleUnit.DEGREES);
    
    double moveSpeed = -5.5D;
    double moveStart = this.globalTime.seconds();
    
    strafeSpeed(moveSpeed);
    
    double carouselVelocity = expectedVelocity;
    startTime = this.globalTime.seconds();
    
    double distanceTravelled = 0;
    double lastTime = startTime;
    
    while (this.globalTime.seconds() < startTime + 1.0D && opModeIsActive());
    
    startTime = this.globalTime.seconds();
    
    do {
      carouselVelocity = this.carousel.getVelocity(AngleUnit.DEGREES);
      
      //double vel = ticksToVelocity(this.backLeft.getVelocity());
      
      //double elapsed =this.globalTime.seconds();
      
      //double dif = elapsed - lastTime;
      //lastTime = elapsed;
      
      //distanceTravelled += vel * dif;
    } while (approximatelyEqual(expectedVelocity, carouselVelocity, velocityThreshold) && opModeIsActive());
    
    telemetry.addData("caoursel velcoity", carouselVelocity);
    telemetry.update();
    
    double finished = this.globalTime.seconds();
    
    distanceTravelled = moveSpeed * (finished - startTime) * strafeMultiplier;
    
    this.backLeft.setVelocity(0.0D);
    this.backRight.setVelocity(0.0D);
    this.frontLeft.setVelocity(0.0D);
    this.frontRight.setVelocity(0.0D);
    
    startTime = finished;
    
    double runTime = waitTime;
    
    while (this.globalTime.seconds() < startTime + runTime && opModeIsActive());
    
    return distanceTravelled;
  }
  
  /**
    * @brief Convert degrees to radians
    *
    * @param degrees double value in degrees to convert to radians
    * @return double value of radians from converted degrees
  */
  public double toRadians(double degrees) {
    return degrees * Math.PI / 180.0D;
  }
  
  /**
    * @brief Convert radians to degrees
    *
    * @param degrees double value in radians to convert to degrees
    * @return double value of degrees from converted radians
  */
  public double toDegrees(double radians) {
    return radians * 180.0D / Math.PI;
  }
  
  /**
    * @brief Bring arm to a certain level, with a certain offset, at a certain power
    *
    * @pre int level is between 0 and this.armPositions.length (TODO: internal check)
    *
    * @param level level to bring arm to, (index of position in armPositions)
    * @param offset offset, in ticks, for arm motor
    * @param power, from 0-1, that arm motor should be given
  */
  public void setArmLevel(int level, int offset, double power) {
    this.armMotor.setTargetPosition(this.armPositions[level] + offset);
    this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.armMotor.setPower(power);
  }
  
  /**
    * @brief Set raw position of arm in ticks
    *
    * @param position position to go to, in ticks
    * @param power power, from 0-1, to give arm motor
  */
  public void setArmPosition(int position, double power){
    this.armMotor.setTargetPosition(position);
    this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.armMotor.setPower(power);
  }
  
  /**
    * @brief Return a random final message from finalMessages
    *
    * @return String final message
  */
  public String getRandomFinalMessage(){
    int index = (int)(Math.random() * finalMessages.length);
    
    return finalMessages[index];
  }
  
  /**
    * @brief Block execution for a period of time
    *
    * @param time time to block execution for
  */
  public void wait(double time){
    // get start time
    double start = this.globalTime.seconds();
    
    // wait until end time
    while(this.globalTime.seconds() <= start+time && opModeIsActive());
  }

  /**
    TODO:
    
    ADDRESS ALL OTHER TODOS

    ADD AUTO FUNCTIONS:
      -primitive strafe+forward at same time (for new auto w/ redesigned arm)
    
    ADD TELEOP/MULTIPURPOSE FUNCTIONS:
      -get function structures set up for each of following
      -function to move new arm to proper position
      -function to swivel arm to proper angle (note: different angles for different cube spots?)
      -function to spin intake
      -function to get distance/color sensor detections for block
    
    STRUCTURE MAIN TELEOP:
      -program new control scheme w/ dummy functions
  */
}
