package AutoTools;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
  
  public DcMotorEx intakeMotor;
  public DcMotorEx swivelMotor;
  
  public Servo clawServoLeft;
  public Servo clawServoRight;
  
  public DistanceSensor freightSensor;
  
  private BNO055IMU imu;
  
  // NOTE: uninitialized by initVals, exist only so that old opmodes compile.
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
  
  // NOTE: this shouldn't have to exist
  public double frictionConstant = 24.0/23.5625; // factor to increase distance by based on chain friction/tension
  
  public double botWidth = 9.0;
  public double botLength = 13.25;
  public double pivotConstant = 180.0/127.3125; // value to multiply pivot degrees by to pivot properly
  
  public double strafeMultiplier = 1.045;
  
  public double autoSpeed = 40.0D;
  
  public double blockingPauseTime = 0.0;
  
  public double startingOrientation = 0.0D;
  public double intendedRotation = 0.0D;
  
  public double x;
  public double y;
  
  public String[] finalMessages = new String[] { "mission accomplished", "payload delivered", "job well done", "pheonix is a genius", "tyler is better than saeid", "the cake is a lie", "the Baked Ziti King reigns supreme"};
  
  public double[] armPositions = new double[]{-5, 15, 42, 69, 75};
  
  public int armOffset = 0;
  
  // NEW STUFF //
  public DcMotorEx[] driveMotors;
  
  // SETTINGS (can be changed throughout teleop to affect bot behavior) //
    
  // drive mode
  public int driveMode = 0; // 0 - automatic 1 - manual
  
  // desired claw state
  public boolean clamped = false; // false - unclamped, true - clamped
  
  // swivel
  public double swivelOffset = 0;
  public double swivelRestingPosition; // resting position of swivel, as an angle
  public double swivelOutPosition; // out position of swivel, as an angle
  public double swivelWaitPosition; // angle to wait for arm to go to prope-r position
  public double swivelReturningPosition; // angle to wait for arm when returning
  public double desiredSwivelRotation = 0; // current rotation of the swivel
  
  public double swivelGearRatio = 2.0/1.0;
  public double swivelTpr = 1425.1 * this.swivelGearRatio;
  public double swivelTolerance = 1.5;
  
  // arm
  public double armRestingPosition = 4; // angle to idle for the arm
  public double armWaitPosition = 9; // angle to be at for the swivel to rotate properly
  public double armDownPosition = 21; // angle to be at for depositing on shared
  public double armUpPosition = 69; // angle to be at to reach top hub level
  public double armCapPosition = 85;
  public double desiredArmPosition = this.armRestingPosition;
  
  public double armGearRatio = 1.0/1.0;
  public double armTpr = 1425.1 * this.armGearRatio;
  public double armTolerance = 3.0;
  
  // intake
  public double intakePower = 0; // 0 - off, -1 - backwards, 1 - forwards
  public double maxIntakePower = 0.65;
  public double intakeTpr = 537.7; // no gear ratio
  public double intakeTolerance = 1; // intake position tolerance in degrees
  public boolean intakeCentering = false; // if intake is centering
  public double distanceThreshold = 40.0; // millimeters away from the distance sensor freight has to be to be grabbed
  
  // claw
  public double depositWaitTime = 0.15; // time to wait for freight to deposit before returning
   
  // STATES (values which change depending on bot state) //
  
  /**
   * @brief State the automated cycle is in
   * 
   * See checkCycleState<Alliance> functions for details
   */
  public int cycleState = 0;
  public int lastCycleState = 0;
  public double lastCycleStateChange = 0;
  
  // swivel
  public double swivelRotation = 0; // rotation of the swivel, in degrees
  public int depositSide = 0; // 0 = left, 1 = right
  
  // arm
  public double armPosition = 0; // rotation of the arm, in degrees
  
  // claw
  public boolean freightInClaw = false; // if freight is in the claw, based on distance sensor readings
  
  /**
   * @brief Set all swivel values
   * 
   * @note can be overriden by inheriting classes to change values for other alliances
   * @note values are for blue alliance shared shipping hub by default
   */
  public void initializeSwivel(){
    this.blueSwivelShared();
  }
  
  /**
   * @brief Set swivel to go to opposite direction
   * 
   * @note can be overriden by inherting classes to change values for other alliances
   */
  public void reverseSwivel(){
    this.blueSwivelAlliance();
  }
  
  /**
   * @brief Check if swivel is busy (NOT using isBusy())
   */
  public boolean isSwivelBusy(){
    return !this.approximatelyEqual(this.swivelRotation - this.swivelOffset, this.desiredSwivelRotation, this.swivelTolerance);
  } 
  
  /**
   * @brief Check if arm is busy (NOT using isBusy())
   */
  public boolean isArmBusy(){
    return !this.approximatelyEqual(this.armPosition, this.desiredArmPosition, this.armTolerance);
  } 
  
  /**
   * @brief Check if intake is busy (NOT using isBusy())
   * 
   * @note Only works if checking if intake is at its intended position
   */
  public boolean isIntakeBusy(){
    return !this.approximatelyEqual(this.intakeMotor.getCurrentPosition(), this.intakeMotor.getTargetPosition(), this.intakeTolerance / 360.0 * this.intakeTpr);
  }
   
  /**
   * @brief Assign swivel values for blue alliance + shared shipping hub
   */
  public void blueSwivelShared(){
    this.swivelRestingPosition = 0; // resting position of swivel, as an angle
    this.swivelOutPosition = 115; // out position of swivel, as an angle
    this.swivelWaitPosition = 18; // angle to wait for arm to go to proper position
    this.swivelReturningPosition = 90; // angle to wait for arm when returning
    
    this.depositSide = 1;
  }
  
  /**
   * @brief Assign swivel values for blue alliance + alliance shipping hub
   * 
   * @note Same as calling redSwivelShared, but more self-documenting
   */
  public void blueSwivelAlliance(){
    this.redSwivelShared();
  }
  
  /**
   * @brief Assign swivel values for red alliance + shared shipping hub
   */
  public void redSwivelShared(){
    this.blueSwivelShared();
    
    this.depositSide = 0;
    
    this.swivelRestingPosition = -this.swivelRestingPosition; // resting position of swivel, as an angle
    this.swivelOutPosition = -this.swivelOutPosition; // out position of swivel, as an angle
    this.swivelWaitPosition = -this.swivelWaitPosition; // angle to wait for arm to go to proper position
    this.swivelReturningPosition = -this.swivelReturningPosition; // angle to wait for arm when returning
  }
  
  /**
   * @brief Assign swivel values for red alliance + alliance shipping hub
   * 
   * @note Same as calling blueSwivelShared, but more self-documenting
   */
  public void redSwivelAlliance(){
    this.blueSwivelShared();
  }
  
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
    //this.tapeBlue = (CRServo)this.hardwareMap.get(CRServo.class, "tapeBlue");
    //this.tapeRed = (CRServo)this.hardwareMap.get(CRServo.class, "tapeRed");
    this.clawServoLeft = (Servo)this.hardwareMap.get(Servo.class, "clawServoLeft");
    this.clawServoRight = (Servo)this.hardwareMap.get(Servo.class, "clawServoRight");
    this.imu = (BNO055IMU)this.hardwareMap.get(BNO055IMU.class, "imu");
    this.intakeMotor = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "intakeMotor");
    this.swivelMotor = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "swivelMotor");
    this.freightSensor = (DistanceSensor)this.hardwareMap.get(DistanceSensor.class, "freightSensor");
  
    //this.driveMotors = new DcMotorEx[]{this.frontLeft, this.frontRight, this.backLeft, this.backRight};
    this.driveMotors = new DcMotorEx[]{this.backRight, this.backLeft, this.frontRight, this.frontLeft};
  
    this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }
  
  /**
    * @brief Initialize arm PIDF and offset
    *
    * Initialize the arm's PIDF coefficients and apply the arm offset to the global arm positions
  */
  public void initArm() {
    // reset swivel
    this.swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    
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
    this.frontLeft.setTargetPosition((int)fl);
    this.frontRight.setTargetPosition((int)fr);
    this.backLeft.setTargetPosition((int)bl);
    this.backRight.setTargetPosition((int)br);
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
   * @brief Checks if all drive motors are at their positions
   * 
   * @return boolean false if not done, true if done
   */
  public boolean isDone(){
    return !( this.backLeft.isBusy() || this.backRight.isBusy() || this.frontLeft.isBusy() || this.frontRight.isBusy() );
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
  */
  public void unclamp() {
    this.clawServoLeft.setPosition(0.1D);
    this.clawServoRight.setPosition(0.35D);
  }
  
  /**
    * @brief Fully clamp the claw servos to grab freight
    *
    * @todo Same todos as this.unclamp()
  */
  public void clamp() {
    this.clawServoLeft.setPosition(0.0525D);
    this.clawServoRight.setPosition(0.44D);
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
    * @note DEPRECATED, use ticksToInches instead
    * @brief Convert ticks/s to in/s
    *
    * @param ticks tpr to convert to in/s
    * 
    * @return velocity in in/s
  */
  public double ticksToVelocity(double ticks){
    double revolutions = ticks / this.tpr;
    double velocity = revolutions * this.wheelCircumference;
    
    return velocity;
  }
  
  /**
    * @note DEPRECATED, use inchesToTicks instead
    * @brief Convert in/s to ticks/s
    *
    * @param velocity in/s to convert to tpr
    *
    * @return velocity in tpr
  */
  public double velocityToTicks(double velocity){
    double revolutions = velocity / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    
    return ticks;
  }
  
  /**
   * @brief Convert drive ticks to inches
   * 
   * @param ticks double to convert from ticks to inches
   * @return inches as double
   */
  public double ticksToInches(double ticks){
    double revolutions = ticks / this.tpr;
    double inches = revolutions * this.wheelCircumference;
    
    inches /= this.frictionConstant;
    
    return inches;
  }
  
  /**
   * @brief Convert inches to drive ticks
   * 
   * @note don't use for anything other than drive motors, as it uses drive TPR and other motors might have different TPR and gear ratios
   *  
   * @param inches double to convert from inches to ticks
   * @return ticks as double
   */
  public double inchesToTicks(double inches){
    inches *= this.frictionConstant;
    
    double revolutions = inches / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
      
    return ticks;
  }
  
  public void goForwardDistanceOld(double distance, double speed, boolean blocking) {
    speed *= this.autoSpeed;
    double startingOrientation = getYRotation();
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
    if (blocking)
      while (opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy()); 
    double rotation = -this.intendedRotation;
    if (Math.abs(getYRotation() - this.intendedRotation) > 3.5D)
      rotation = -getYRotation(); 
    this.x += Math.sin(toRadians(rotation)) * distance;
    this.y += Math.cos(toRadians(rotation)) * distance;
    //this.telemetry.addData("x", Double.valueOf(this.x));
    //this.telemetry.addData("y", Double.valueOf(this.y));
    //this.telemetry.addData("orientation used", Double.valueOf(rotation));
    //this.telemetry.update();
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
    double ticks = inchesToTicks(distance);
    
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
    double ticks = inchesToTicks(distance);
    
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
   * @brief Perform trapezoid velocity check on drive train for one cycle
   * 
   * Useful for main loop if active computations are required w/o blocking
   * 
   * @TODO use in trapzeoidVelocity
   * 
   * @param velocity velocity in in/s
   * @param acceleration (approximate) acceleration in in/s
   */
  public void trapezoidVelocityCycle(double velocity, double acceleration, double delta){
    // convert to ticks
    velocity = this.inchesToTicks(velocity);
    acceleration = this.inchesToTicks(acceleration);
    
    double MAX_ACCELERATION = acceleration;
    double MIN_DECCELERATION = this.inchesToTicks(24.0); // ideal decceleration rate TODO: global
    double DECCELERATATION_POINT = this.inchesToTicks(0.0); // velocity that profile should deccelerate towards
    
    double DECCELERATION_POINT2 = DECCELERATATION_POINT*DECCELERATATION_POINT;
    
    double[] velocities = new double[4];
    
    // run trapezoid profile for each motor
    for(int i = 0; i < driveMotors.length; i++){
      DcMotorEx motor = driveMotors[i];
      
      double currentVelocity = Math.abs(motor.getVelocity());
      
      double positionError = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
      
      double outputVelocity, outputAcceleration;
      
      //if(positionError < 0) directionMultiplier = -1;
      
      if( Math.abs(currentVelocity) < velocity){
        outputVelocity = currentVelocity + MAX_ACCELERATION * delta;
        outputAcceleration = MAX_ACCELERATION;
      } else {
        outputVelocity = velocity;
        outputAcceleration = 0;
      }
      
      // d^2 = vi^2 + 2ax
      // (d^2 - vi^2) / 2a
      // (d^2 - vi^2) / 2x
      
      double outputVelocity2 = outputVelocity*outputVelocity;
      
      if( positionError <= (DECCELERATION_POINT2 - outputVelocity2) / (2 * -MIN_DECCELERATION) ){
        // recalculate decceleration based on distance to target
        double decceleration = (DECCELERATION_POINT2 - outputVelocity2) / (2*positionError);
        
        // check if decceleration is infinity
        if(2*positionError == 0){
          // set decceleration to 0
          decceleration = 0;
        }
        
        outputVelocity = currentVelocity + decceleration * delta;
        outputAcceleration = decceleration;
      }
      
      /*telemetry.addData("positionError", this.ticksToInches(positionError));
      telemetry.addData("currentVelocity", this.ticksToInches(currentVelocity));
      telemetry.addData("outputVelocity", this.ticksToInches(outputVelocity));
      telemetry.addData("velocity", this.ticksToInches(velocity));
      telemetry.addData("outputAcceleration", this.ticksToInches(outputAcceleration));
      telemetry.update();*/
      
      velocities[i] = outputVelocity;
      //motor.setVelocity(outputVelocity);
    }
    
    for(int i = 0; i < driveMotors.length; i++){
      driveMotors[i].setVelocity(velocities[i]);
    }
  }
  
  /**
   * @brief Maintain the velocity of the motors via trapezoid motion profile
   * 
   * @param velocity velocity, in in/s, to travel at
   * @param acceleration (approximate) acceleration, in in/s
   */
  public void trapezoidVelocity(double velocity, double acceleration){
    // convert to ticks
    velocity = this.inchesToTicks(velocity);
    acceleration = this.inchesToTicks(acceleration);
    
    double MAX_ACCELERATION = acceleration;
    double MIN_DECCELERATION = this.inchesToTicks(24.0); // ideal decceleration rate TODO: global
    double DECCELERATATION_POINT = this.inchesToTicks(0.0); // velocity that profile should deccelerate towards
    
    double DECCELERATION_POINT2 = DECCELERATATION_POINT*DECCELERATATION_POINT;
    
    // run trapezoid profile for each motor
    
    double currentTime = globalTime.seconds();
    double previousTime = currentTime;
    double delta = 0;
    
    while(!isDone()){
      currentTime = globalTime.seconds();
      delta = currentTime - previousTime;
      previousTime = currentTime;
      
      for(int i = 0; i < driveMotors.length; i++){
        DcMotorEx motor = driveMotors[i];
        
        double currentVelocity = Math.abs(motor.getVelocity());
        
        double positionError = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        
        double outputVelocity, outputAcceleration;
        
        //if(positionError < 0) directionMultiplier = -1;
        
        if( Math.abs(currentVelocity) < velocity){
          outputVelocity = currentVelocity + MAX_ACCELERATION * delta;
          outputAcceleration = MAX_ACCELERATION;
        } else {
          outputVelocity = velocity;
          outputAcceleration = 0;
        }
        
        // d^2 = vi^2 + 2ax
        // (d^2 - vi^2) / 2a
        // (d^2 - vi^2) / 2x
        
        double outputVelocity2 = outputVelocity*outputVelocity;
        
        if( positionError <= (DECCELERATION_POINT2 - outputVelocity2) / (2 * -MIN_DECCELERATION) ){
          // recalculate decceleration based on distance to target
          double decceleration = (DECCELERATION_POINT2 - outputVelocity2) / (2*positionError);
          
          // check if decceleration is infinity
          if(2*positionError == 0){
            // set decceleration to 0
            decceleration = 0;
          }
          
          outputVelocity = currentVelocity + decceleration * delta;
          outputAcceleration = decceleration;
        }
        
        telemetry.addData("positionError", this.ticksToInches(positionError));
        telemetry.addData("currentVelocity", this.ticksToInches(currentVelocity));
        telemetry.addData("outputVelocity", this.ticksToInches(outputVelocity));
        telemetry.addData("velocity", this.ticksToInches(velocity));
        telemetry.addData("outputAcceleration", this.ticksToInches(outputAcceleration));
        telemetry.update();
        
        motor.setVelocity(outputVelocity);
      }
    }
  }
  
  /**
   * @brief Go forward some distance, velocity and acceleration using trapezoid motion profile
   *  
   * @note copied pretty much entirely from https://gm0.org/en/latest/docs/software/control-loops.html , thanks for that mr/mrs. author of that article
   * 
   * @param distance distance to travel, in inches
   * @param velocity maximum allowed velocity over course of movement, in inches per second
   * @param acceleration maximum allowed acceleration over course of movement, in inches per second
   */
  public void goDistanceTrapezoid(double distance, double velocity, double acceleration){
    // convert intended distance and velocity to ticks
    distance = this.inchesToTicks(distance);
    
    // go to position
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.setPosition((int)distance);
    
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // maintain velocity through trapezoid profile
    this.trapezoidVelocity(velocity, acceleration);
  }
  
  /**
   * @brief Tell the motors to go forward a distance, but don't assign velocity
   * 
   * Useful with trapezoidVelocityCycle if you don't want to block execution in main loop
   * 
   * @param distance distance in inches
   */
  public void goDistanceNoVelocity(double distance){
    // convert intended distance and velocity to ticks
    distance = this.inchesToTicks(distance);
    
    // go to position
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.setPosition((int)distance);
    
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    * @note DEPRECATED, use compoundMove instead
    * @todo delete this
    * 
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
   * @brief Move forward and strafe a certain distance at the same time
   * 
   * It's recommended to use this in place of any calls to moving forward or strafing.
   * Even if only moving one direction, use anyways since it'll be easier when cleaning up old functions later on.
   * 
   * @note Even though not indicated by the function name, this ALWAYS uses trapezoid velocity
   * 
   * @param forward inches to move forward
   * @param strafe inches to strafe
   * @param velocity velocity to move at, in in/s
   * @param acceleration (approximate) acceleration in in/s
  */
  public void compoundMove(double forward, double strafe, double velocity, double acceleration){
    velocity = this.inchesToTicks(velocity);
    
    double fl = this.inchesToTicks(forward + strafe);
    double fr = this.inchesToTicks(forward - strafe);
    double bl = this.inchesToTicks(forward - strafe);
    double br = this.inchesToTicks(forward + strafe);
    
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.setPosition((int)fl, (int)fr, (int)bl, (int)br);
    
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    this.setVelocity(velocity);
    
    //this.trapezoidVelocity(velocity, acceleration);
  }
  
  /**
   * @brief Move forward and strafe a certain distance at the same time, in a straight line
   * 
   */
  public void compoundMoveStraight(double forward, double strafe, double velocity, double acceleration){
    velocity = this.inchesToTicks(velocity);
    
    double fl = this.inchesToTicks(forward + strafe);
    double fr = this.inchesToTicks(forward - strafe);
    double bl = this.inchesToTicks(forward - strafe);
    double br = this.inchesToTicks(forward + strafe);
    
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.setPosition((int)fl, (int)fr, (int)bl, (int)br);
    
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    double ratio = Math.abs(fl)/Math.abs(fr);
    double ratio2 = Math.abs(br)/Math.abs(bl);
    
    this.setVelocity(velocity * ratio, velocity, velocity, velocity * ratio);
  }
  
  /**
   * @brief Move forward and strafe a certain distance at the same time, but don't assign velocity
   * 
   * Useful with trapezoidVelocityCycle if you don't want to block execution in main loop
   * 
   * @param forward distance in inches to move forward
   * @param strafe distance in inches to strafe
   */
  public void compoundMoveNoVelocity(double forward, double strafe){
    double fl = this.inchesToTicks(forward + strafe);
    double fr = this.inchesToTicks(forward - strafe);
    double bl = this.inchesToTicks(forward - strafe);
    double br = this.inchesToTicks(forward + strafe);
    
    this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    this.setPosition((int)fl, (int)fr, (int)bl, (int)br);
    
    this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  /**
    * @note DEPRECATED, use pivot instead
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
   * @brief Pivot the bot by an angle
   * 
   * Pivot the bot by an angle using only predicted ticks.
   * 
   * @pre this.botWidth and this.botLength are set to the distances between each pair of mecanum wheels, width-wise and length-wise
   *
   * @param degrees degrees to rotate, as a double
   * @param speed approximate speed to rotate (slightly off)
   * @TODO fix speed parameter
   */
  public void pivot(double degrees, double speed){
      // add to intended rotation
      this.intendedRotation += degrees;
      
      // convert velocity to ticks
      speed = this.inchesToTicks(speed);
      
      // multiply angle by 15/11 ?
      // TODO: figure out why
      // best guess: it was incorrect to assume that the amount of times the wheel spins
      // is directly related to distance it travels, as is the case for forward movement
      // and likely spins ~11/15 less as a result
      // How this mathematically makes sense, I have no idea
      degrees *= this.pivotConstant;
      
      // get predicted distance
      double radians = this.toRadians(degrees);
      
      // radians * this.radius
      double radius = Math.sqrt((this.botWidth*this.botWidth + this.botLength*this.botLength)/4);
      double distance = radians * radius;
      
      // convert distance to ticks
      /*double revolutions = distance / this.wheelCircumference;
      double ticks = revolutions * this.tpr;*/
      double ticks = this.inchesToTicks(distance);
      
      // set position
      this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      this.frontLeft.setTargetPosition((int)ticks);
      this.backLeft.setTargetPosition((int)ticks);
      this.frontRight.setTargetPosition(-(int)ticks);
      this.backRight.setTargetPosition(-(int)ticks);
      
      this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      this.setVelocity(speed);
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
    this.armMotor.setTargetPosition((int)this.armPositions[level] + offset);
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
   * @brief Set the angle of the swivel based on ticks 
   * 
   * @param angle angle to rotate swivel to, in degrees
  */
  public void setSwivelAngle(double angle){
    double rotation = angle + this.swivelOffset;
    
    // calculate rotation in ticks
    double ticks = (rotation/360.0) * this.swivelTpr;
    
    // set swivel position to ticks
    this.swivelMotor.setTargetPosition((int)ticks);
    
    this.swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  
    this.swivelMotor.setPower(0.85);
  }
  
  /**
   * @brief return the rotation of the swivel
   */
  public double getSwivelRotation(){
    return (this.swivelMotor.getCurrentPosition() / this.swivelTpr) * 360.0;  
  }
   
  /**
   * @brief Set the rotation of the arm
   */
  public void setArmRotation(double rotation){
    // calculate rotation in ticks
    double ticks = (rotation/360.0) * this.armTpr;
    
    // set swivel position to ticks
    this.armMotor.setTargetPosition((int)ticks);
    
    this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  
    this.armMotor.setPower(0.4);
  }
  
  /**
   * @brief Return the current rotation of the arm in degrees
   */
  public double getArmRotation(){
    return (this.armMotor.getCurrentPosition() / this.armTpr) * 360.0;
  }
  
  /**
   * @brief Updates the intake power based on this.intakePower
   * 
   */
  public void updateIntake(){
    // check if intake is currently centering, if not update power
    if(this.intakeCentering){
      // if intake motor is at position, set mode to run w/o encoder & update centering
      if(!this.isIntakeBusy() && !this.intakeMotor.isBusy()){
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        this.intakeCentering = false;
        
        this.intakeMotor.setPower(0);
      }
    } else {
      // TODO: make global relative power
      this.intakeMotor.setPower(this.intakePower * this.maxIntakePower);
    }
  }
  
  /**
   * @brief Bring intake to straight up and down
   */
  public void centerIntake(){
    // determine current position of intake
    double currentRotation = (double)this.intakeMotor.getCurrentPosition() / this.intakeTpr * 360;
    
    // get required correction
    //  -(r % 180) 
    
    // check which correction is faster
    double offset = currentRotation % 180;
    
    double correction = -offset;
    
    if(offset > 115){
      correction = 180 - Math.abs(offset);  
    }
    
    /*if( (180 - Math.abs(offset) ) < Math.abs(offset) ){
      correction = -(180 - Math.abs(offset));
    }*/
    
    // convert correction to ticks
    double tickCorrection = correction / 360 * this.intakeTpr;
    
    tickCorrection += this.intakeMotor.getCurrentPosition();
    
    // correct
    this.intakeMotor.setTargetPosition((int)tickCorrection);
    
    this.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    this.intakeMotor.setPower(0.25);
    
    this.intakeCentering = true;
  }
  
  /**
   * @brief Check if freight is in claw and update appropriate state
   */ 
  public void updateFreightInClaw(){
    double currentDistance = this.freightSensor.getDistance(DistanceUnit.MM);
    
    this.freightInClaw = currentDistance <= this.distanceThreshold;
  }
  
  /**
   * @brief Update claw based on clamp state
   */
  public void updateClaw(){
    if(this.clamped)
      this.clamp();
    else
      this.unclamp();
  }
  
  /**
   * @brief Check the current cycle state and see if anything needs to change, for blue alliance
   *
   * Expected to be ran after updateStates, but before checkSettings
   * 
   * See this.cycleState documentation for more info
   */
  public void checkCycleState(){
    switch(this.cycleState){
      // at rest, allow intake to run and check for freight
      // if freight is found, move swivel to "wait" position
      case 0: {
        // unclamp by default
        this.clamped = false;
        
        // check for swivel value switch
        if(gamepad2.left_trigger > 0.3){
          this.reverseSwivel();
        } else {
          this.initializeSwivel();
        }
        
        // adjustment override
        if(gamepad2.right_bumper){
          this.swivelOffset += gamepad2.left_stick_x;
          
          break;
        }
        
        // check for freight (or for gamepad2 override)
        if( (this.freightInClaw || gamepad2.a) && !this.isSwivelBusy() ){
          // clamp claw
          this.clamped = true;
          
          // center intake
          this.centerIntake();
          
          // update cycle state
          this.cycleState++;
        }
        
        break;
      }
      
      // wait for intake to center
      case 1: {
        // disable intake
        this.intakePower = 0;
        
        if(!this.intakeCentering){
          this.cycleState++;
        }
        
        break;
      }
      
      case 2: {
        // disable intake
        this.intakePower = 0;
        
        // set arm position to wait position whene clamped
        if(this.globalTime.seconds() >= this.lastCycleStateChange+0.1){
          this.desiredArmPosition = this.armWaitPosition;
          
          this.cycleState++;
        }
        
        break;
      }
      
      // we have freight
      // once the swivel is done, move arm to "wait" position.  once done, move swivel to "out" position
      case 3: {
        // disable intake
        this.intakePower = 0;
        
        // check if swivel is (roughly) at position
        if( !this.isArmBusy() ){
          // set desired swivel rotation to out position
          this.desiredSwivelRotation = this.swivelOutPosition;
          
          // set arm position
          this.desiredArmPosition = this.armDownPosition;
          
          // update cycle state
          this.cycleState++;
        }
        
        break;
      }
      
      // swivel is at position, waiting for deposit command
      // also check for arm input & move arm to appropriate position
      case 4: {
        // await the Baked Ziti King's command
        if(gamepad2.a){
          // deposit!
          this.clamped = false;
          
          // update cycle state
          this.cycleState++;
        }
        
        // if swivel is out far enough, reverse intake
        if(!this.isSwivelBusy()){
          // reverse intake
          this.intakePower = -1;
        } else {
          this.intakePower = 0;
        }
        
        // check for arm input
        if(gamepad2.left_stick_y != 0 && !this.isArmBusy() ){
          // if at arm wait, go to armUp
          if(-gamepad2.left_stick_y > 0){
            this.desiredArmPosition = this.armUpPosition;
          // if arm at up, go to armWait
          } else if(-gamepad2.left_stick_y < 0){
            this.desiredArmPosition = this.armDownPosition;
          }
        }
        
        if(gamepad2.right_stick_y != 0){
          this.desiredArmPosition += gamepad2.right_stick_y;  
        }
        
        // check for swivel input
        if(gamepad2.left_stick_x != 0){
          this.desiredSwivelRotation += -gamepad2.left_stick_x*2;
        }
        
        break;
      }
      
      // waiting for full deposit
      // NOTE: probably doesn't need its own cycle state, but is good for neatness or whatever
      case 5: {
        this.intakePower = 0;
        
        // recenter intake
        this.centerIntake();
        
        // wait for depositWaitTime seconds
        // this convienently disables the drive train as well, which is actual desired here so that lucas doesn't have to worry about timing
        this.wait(this.depositWaitTime);
        
        // update state
        this.cycleState++;
        
        break;
      }
      
      // if aux is holding down a trigger, or intake is centering, wait until release to move arm
      case 6: {
        if(gamepad2.left_trigger < 0.3 && !this.intakeCentering){
          // set desired arm position to wait
          this.desiredArmPosition = this.armWaitPosition+7;
          this.cycleState++;
        }
        
        break;  
      }
      
      // we've deposited
      // bring swivel back
      case 7: {
        this.intakePower = 0;
        
        // once arm is at wait, bring swivel back
        if(!this.isArmBusy()){
          this.desiredSwivelRotation = this.swivelRestingPosition;
          
          this.cycleState++;
        }
        
        break;
      }
      
      // once swivel has arrived, drop arm down and reset
      case 8: {
        this.intakePower = 0;
        
        // check is swivel is at position
        if( !this.isSwivelBusy() ){
          // move arm to position
          this.desiredArmPosition = this.armRestingPosition;
          
          this.clamped = false;
          
          // reset cycle state
          this.cycleState = 0;
        }
        
        break;
      }
    }
  }
  
  /**
   * @brief Check the current cycle state and see if anything needs to change, for red alliance
   *
   * Expected to be ran after updateStates, but before checkSettings
   * 
   * See this.cycleState documentation for more info
   */
  public void checkCycleStateRed(){
    switch(this.cycleState){
      // at rest, allow intake to run and check for freight
      // if freight is found, move swivel to "wait" position
      case 0: {
        // unclamp by default
        this.clamped = false;
        
        // check for swivel value switch
        if(gamepad2.right_trigger > 0.3){
          this.redSwivelAlliance();
        } else {
          this.redSwivelShared();
        }
        
        // check for freight (or for gamepad2 override)
        if( (this.freightInClaw || gamepad1.a) && !this.isSwivelBusy() ){
          // clamp claw
          this.clamped = true;
          
          // center intake
          this.centerIntake();
          
          // update cycle state
          this.cycleState++;
        }
        
        break;
      }
      
      // wait for intake to center
      case 1: {
        if(!this.intakeCentering){
          this.cycleState++;
        }
        
        break;
      }
      
      case 2: {
        // disable intake
        this.intakePower = 0;
        
        // set arm position to wait position whene clamped
        if(this.globalTime.seconds() >= this.lastCycleStateChange+0.1){
          this.desiredArmPosition = this.armWaitPosition;
          
          this.cycleState++;
        }
        
        break;
      }
      
      // we have freight
      // once the swivel is done, move arm to "wait" position.  once done, move swivel to "out" position
      case 3: {
        // disable intake
        this.intakePower = 0;
        
        // check if swivel is (roughly) at position
        if( !this.isArmBusy() ){
          // set desired swivel rotation to out position
          this.desiredSwivelRotation = this.swivelOutPosition;
          
          // update cycle state
          this.cycleState++;
        }
        
        break;
      }
      
      // swivel is at position, waiting for deposit command
      // also check for arm input & move arm to appropriate position
      case 4: {
        // await the Baked Ziti King's command
        if(gamepad1.a){
          // deposit!
          this.clamped = false;
          
          // update cycle state
          this.cycleState++;
        }
        
        // if swivel is out far enough, reverse intake
        if(this.swivelRotation < this.swivelWaitPosition){
          // reverse intake
          this.intakePower = -1;
        } else {
          this.intakePower = 0;
        }
        
        // check for arm input
        if(gamepad2.left_stick_y != 0 && !this.isArmBusy() ){
          // if at arm wait, go to armUp
          if(this.desiredArmPosition == this.armWaitPosition && -gamepad2.left_stick_y > 0){
            this.desiredArmPosition = this.armUpPosition;
          // if arm at up, go to armWait
          } else if(this.desiredArmPosition == this.armUpPosition && -gamepad2.left_stick_y < 0){
            this.desiredArmPosition = this.armWaitPosition;
          }
        }
        
        break;
      }
      
      // waiting for full deposit
      // NOTE: probably doesn't need its own cycle state, but is good for neatness or whatever
      case 5: {
        // recenter intake
        this.centerIntake();
        
        // wait for depositWaitTime seconds
        // this convienently disables the drive train as well, which is actual desired here so that lucas doesn't have to worry about timing
        this.wait(this.depositWaitTime);
        
        // set desired arm position to wait
        this.desiredArmPosition = this.armWaitPosition;
        
        // update state
        this.cycleState++;
        
        break;
      }
      
      // we've deposited
      // bring swivel back
      case 6: {
        // once arm is at wait, bring swivel back
        if(!this.isArmBusy()){
          this.desiredSwivelRotation = this.swivelRestingPosition;
          
          this.cycleState++;
        }
        
        break;
      }
      
      // once swivel has arrived, drop arm down and reset
      case 7: {
        // check is swivel is at position
        if( !this.isSwivelBusy() ){
          // move arm to position
          this.desiredArmPosition = this.armRestingPosition;
          
          this.clamped = false;
          
          // reset cycle state
          this.cycleState = 0;
        }
        
        break;
      }
    }
  }
  
  /**
   * @brief Update each state to its appropriate value
  */
  public void updateStates(){
    // update block in slot check
    this.updateFreightInClaw();
    
    // swivel rotation
    this.swivelRotation = this.getSwivelRotation();
    
    // arm position
    this.armPosition = this.getArmRotation();
    
    // update last cycled
    if(this.cycleState != this.lastCycleState){
      this.lastCycleStateChange = this.globalTime.seconds();
    }
    
    this.lastCycleState = this.cycleState;
  }
  
  /**
   * @brief Check each current setting and react accordingly 
  */
  public void checkSettings(){
    this.setArmRotation(this.desiredArmPosition);
    
    this.setSwivelAngle(this.desiredSwivelRotation);  
    
    this.updateIntake();
    
    this.updateClaw(); // TODO: fix claw
  }
  
  /**
   * @brief Log each state, without updating telemetry
  */
  public void logStates(){
    this.logStates(false);
  } 
   
  /**
   * @brief Log each state, updating telemetry if update is true
   * 
   * @param update if telemetry should be updated
  */
  public void logStates(boolean update){
    telemetry.addData("swivelRotation", this.swivelRotation);
    telemetry.addData("armRotation", this.armPosition);
    telemetry.addData("freight in claw?", this.freightInClaw + "");
    telemetry.addData("cycleState", String.valueOf(this.cycleState));
    telemetry.addData("intakePosition", this.intakeMotor.getCurrentPosition() / this.intakeTpr * 360);
    telemetry.addData("intake centering?", this.intakeCentering + "");
    telemetry.addData("intakeTarget", this.intakeMotor.getTargetPosition() / this.intakeTpr * 360);
    telemetry.addData("swivel busy?", this.isSwivelBusy());
    
    if(update)
      telemetry.update();
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
