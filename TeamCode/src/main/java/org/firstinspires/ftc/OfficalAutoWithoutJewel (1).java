package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
@Autonomous
public class OfficalAutoWithoutJewel extends LinearOpMode {
    
    //-----------------Robot Controller Camera
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    //====
    
   
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime Timer = new ElapsedTime();

    private Servo Servo1      = null;
    private Servo Servo2     = null;
    private Servo Servo4      = null;
    private Servo Servo5     = null;
    
     private double pivotR;
      private double verticalDrvF;
     private double verticalDrvR;
     private double verticalDrv;
     private double horizontalDrv;
     
     private ColorSensor colorSensor;
    
    private Servo Servo3 = null;
    private DcMotor Extend      = null;
    private DcMotor MotorLift       = null;
    private DcMotor Motor1      = null;
    private DcMotor Motor2      = null;
    private DcMotor Motor3      = null;
    private DcMotor Motor4      = null;
    private double left_posTop    = 0;
    private double right_posTop   = 1.0;
    private double left_posBot    = 0;
    private double right_posBot   = 1.0;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private double ExtendPower;
    private double LiftPower;
    private boolean isScanDone = false;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private int color = 0;
    
    
    private boolean isPosition = true;
    
    private String VuMark = "-";
    
    @Override
    public void runOpMode () {
        
        IntegratingGyroscope gyro;
  ModernRoboticsI2cGyro modernRoboticsI2cGyro; 
   boolean lastResetState = false;
    boolean curResetState  = false;

    // Get a reference to a Modern Robotics gyro object. We use several interfaces
    // on this object to illustrate which interfaces support which functionality.
    modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
   telemetry.log().add("Gyro Calibrating. Do Not Move!");
    modernRoboticsI2cGyro.calibrate();
  
        //---------------Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //South City Robotics 2018 VuforiaLicenseKey 
        //https://developer.vuforia.com/license-manager
        //email darrendotng@gmail.com to obtain username and password to manage license keys
        parameters.vuforiaLicenseKey = "ATbZfEb/////AAAAmQZejrUuP0+4ilV5yBR2nxhz5jMi9FeNdT1dA+wRG/tlwKxFyLVR+WzYN+H7YngjFFE60ojETbbaieEC5Jqa7yJu7qaDsCj6a1biqAafZL6iGrSCE5218seIoIINnYAnWXIGCZD8lSjRPQ0oK6m5cpDdjDF9+ghVmDcBwkDRZk4kRWnYsQIBYlfHDvu6Ct9q6Yso+zQNn2PSrXA2rZg01eDMZz2MS5Fb+Oi9LHKmdG22oXejb6xd4MwxW60m4uxlcDdqcve7wOv9D3oC6y4qY1pjitP1YpQ4tjK0qG285tdMj+1WPIMHJ8/96XOa29CvOjbGXZft71L7rwxy2r+iE3hq/V+0sHCtYB1KOu2pquIx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        
        

        relicTrackables.activate();
        //====
        
        boolean bLedOn = true;
        boolean isOpModeActive = true;
        
        //Ititialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
       Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
     
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
//        SensorServo  = hardwareMap.get(Servo.class, "SensorServo");
        
        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.FORWARD);
        Motor4.setDirection(DcMotor.Direction.FORWARD);
     //   Motor5.setDirection(DcMotor.Direction.FORWARD);
        colorSensor.enableLed(bLedOn);
       
    //    Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            Servo1  = hardwareMap.get(Servo.class, "Servo1");
    Servo2 = hardwareMap.get(Servo.class, "Servo2");
    Servo4  = hardwareMap.get(Servo.class, "Servo4");
    Servo5 = hardwareMap.get(Servo.class, "Servo5");
    
    //Servo3 = hardwareMap.get(Servo.class, "Servo3");
 //   Extend    = hardwareMap.get(DcMotor.class, "MotorExtend");
    MotorLift     = hardwareMap.get(DcMotor.class, "MotorLift");
    
    //Setup
    Servo1.scaleRange(MIN_POS,MAX_POS);
    Servo2.scaleRange(MIN_POS,MAX_POS);
    
    Servo4.scaleRange(MIN_POS,MAX_POS);
    Servo5.scaleRange(MIN_POS,MAX_POS);
    
    Servo1.setPosition(left_posTop);
    Servo2.setPosition(right_posTop);
    
    Servo4.setPosition(right_posBot);
    Servo5.setPosition(left_posBot);
    
    
 //   Servo3.scaleRange(MIN_POS,MAX_POS);
 //   Servo3.setPosition(1.0);
//    Extend.setDirection(DcMotor.Direction.FORWARD);
    MotorLift.setDirection(DcMotor.Direction.REVERSE);
    Motor1.setDirection(DcMotor.Direction.FORWARD);
    Motor2.setDirection(DcMotor.Direction.FORWARD);
    Motor3.setDirection(DcMotor.Direction.FORWARD);
    Motor4.setDirection(DcMotor.Direction.FORWARD);
        
       /* 
        while(runtime.seconds()<30&&isPosition){
        telemetry.addData("akjwdnawkj", Motor5.getCurrentPosition());
        telemetry.update();
        Motor5.setTargetPosition(90);
        Motor5.setPower(.3);
        
        if(Motor5.getCurrentPosition()>10000){
            isPosition = false;
            Motor5.setPower(-.3);
        }
        }
         while(runtime.seconds()<30){
        telemetry.addData("akjwdnawkj", Motor5.getCurrentPosition());
        telemetry.update();
         }
       
        */
        
       
        //Wait for the game to start
        waitForStart();
        runtime.reset();
        
       
        //Gradual movement down so that the sensor doesn't get damaged due to momentum
 //       SensorServo.setPosition(0.2);
 /*
        while ((runtime.seconds() <= 1)&&(opModeIsActive())&&(isOpModeActive == true)) {
             Motor5.setTargetPosition(0);
         Motor5.setPower(.3);
         telemetry.addData("hi:", Motor5.getTargetPosition());
        telemetry.update();
        while(runtime.seconds()<2){
            
        }
        
        while(runtime.seconds()<5){
        
       
 
        Motor5.setTargetPosition(500);
   
        telemetry.addData("hi:", Motor5.getTargetPosition());
        telemetry.update();
        
        
        }
        }
        */
 //       SensorServo.setPosition(0.015);
 Timer.reset();
 while(Timer.seconds()<2){
     Servo4.setPosition(0);
     Servo5.setPosition(1);
 }
 Timer.reset();
 while(Timer.seconds()<.5){
     MotorLift.setPower(0.3);
 }
MotorLift.setPower(0);

Timer.reset();
 while(Timer.seconds()<5){
 RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
               
                VuMark = vuMark.toString();
                if(vuMark.toString() == "RIGHT"){
                telemetry.addData("VUUUIEMARK", VuMark);
                }
                 telemetry.update();

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
              //  telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
 }
 Timer.reset();
    while(Timer.seconds()<2.7){
        Motor4.setPower(-0.2);
        Motor3.setPower(0.2);
    }
     Motor4.setPower(0);
        Motor3.setPower(0);
        Timer.reset();
        while(Timer.seconds()<.3){
            //empty
        }
      

    
    // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
    // A similar approach will work for the Gyroscope interface, if that's all you need.

    // Start calibrating the gyro. This takes a few seconds and is worth performing
    // during the initialization phase at the start of each opMode.
  

    // Wait until the gyro calibration is complete
  
    // Loop until we're asked to stop
   

      // If the A and B buttons are pressed just now, reset Z heading.
      curResetState = (gamepad1.a && gamepad1.b);
      if (curResetState && !lastResetState) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
      }
      lastResetState = curResetState;

      // The raw() methods report the angular rate of change about each of the
      // three axes directly as reported by the underlying sensor IC.
      int rawX = modernRoboticsI2cGyro.rawX();
      int rawY = modernRoboticsI2cGyro.rawY();
      int rawZ = modernRoboticsI2cGyro.rawZ();
      int heading = modernRoboticsI2cGyro.getHeading();
      int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

      // Read dimensionalized data from the gyro. This gyro can report angular velocities
      // about all three axes. Additionally, it internally integrates the Z axis to
      // be able to report an absolute angular Z orientation.
      AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
      float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

      // Read administrative information from the gyro
      int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
      int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

     /* telemetry.addLine()
        .addData("dx", formatRate(rates.xRotationRate))
        .addData("dy", formatRate(rates.yRotationRate))
        .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
      telemetry.addData("angle", "%s deg", formatFloat(zAngle));
      telemetry.addData("heading", "%3d deg", heading);
      telemetry.addData("integrated Z", "%3d", integratedZ);
      telemetry.addLine()
        .addData("rawX", formatRaw(rawX))
        .addData("rawY", formatRaw(rawY))
        .addData("rawZ", formatRaw(rawZ));
      telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
      telemetry.update();
      */
    
  




        while(heading<100){
            heading = modernRoboticsI2cGyro.getHeading();
            telemetry.addData("Heading", heading);
            telemetry.update();
    Motor4.setPower(-0.2);
    Motor3.setPower(-0.2);
    Motor1.setPower(-0.2);
    Motor2.setPower(-0.2);
        }
        while(heading<176){
             heading = modernRoboticsI2cGyro.getHeading();
            telemetry.addData("Heading", heading);
            telemetry.update();
    Motor4.setPower(-0.1);
    Motor3.setPower(-0.1);
    Motor1.setPower(-0.1);
    Motor2.setPower(-0.1);  
        while(heading!=180){
            if(heading<180){
    Motor4.setPower(-0.05);
    Motor3.setPower(-0.05);
    Motor1.setPower(-0.05);
    Motor2.setPower(-0.05);  
            }
            if(heading>180){
    Motor4.setPower(0.05);
    Motor3.setPower(0.05);
    Motor1.setPower(0.05);
    Motor2.setPower(0.05);             
    
            }
        }
        }
        Motor4.setPower(0);
    Motor3.setPower(0);
    Motor1.setPower(0);
    Motor2.setPower(0);
    
 //Motor5.setPower(.3);
/* while(runtime.seconds()<5){
    Motor5.setTargetPosition(605);
    telemetry.addData("Setting Jewel", VuMark);
    telemetry.update();
 }*/
      /*  while ((runtime.seconds() <= 10)&&(opModeIsActive())&&(isOpModeActive == true)&&(isScanDone == false)) {
            telemetry.addData("Scanning", VuMark);
            telemetry.update();
            if(colorSensor.red()>0&&colorSensor.blue()==0){ //if red
            telemetry.addData("Really Scanning", VuMark);
            telemetry.update();
                //pivot robot 
                while(runtime.seconds() <=5.5){ 
                Motor3.setPower(0.1);
                Motor4.setPower(0.1);
                Motor1.setPower(0.1);
                Motor2.setPower(0.1);
                }
                while(runtime.seconds() <=6){
                Motor3.setPower(-0.1);
                Motor4.setPower(-0.1);
                Motor1.setPower(-0.1);
                Motor2.setPower(-0.1);
                }
                Motor3.setPower(0);
                Motor4.setPower(0);
                Motor1.setPower(0);
                Motor2.setPower(0);
                
                isScanDone = true;
            }
            else if(colorSensor.blue()>0&&colorSensor.red()==0){
                telemetry.addData("Really Scanning1", colorSensor.blue());
                telemetry.update();
                color = 1;
                while(runtime.seconds() <=5.50){
                Motor1.setPower(-0.1);
                Motor2.setPower(-0.1);
                Motor3.setPower(-0.1);
                Motor4.setPower(-0.1);
                }
                Motor3.setPower(0);
                Motor4.setPower(0);
                Motor1.setPower(0);
                Motor2.setPower(0);
               // Motor5.setPower(.3);
 */
    

 //currently only works when jewel dectection is blue
 /*while(runtime.seconds()<6)
      //  Motor5.setTargetPosition(0);
                while(runtime.seconds() <= 8){
                Motor3.setPower(-0.3);
                Motor4.setPower(0.3);
               
                }
               Motor3.setPower(0);
                Motor4.setPower(0);
                Motor1.setPower(0);
                Motor2.setPower(0);
                
        */    
        
 //       SensorServo.setPosition(1);
/*
while(runtime.seconds()<5){
    Motor4.setPower(-0.2);
    Motor3.setPower(0.2);
}
        telemetry.addData("Retract", color);*/
        while ((runtime.seconds() <= 30)&&(opModeIsActive())&&(isOpModeActive == true)) {
            
            if(VuMark == "RIGHT"){
                int numOfSpikes = 2;
                while(numOfSpikes < 2){
                    Motor4.setPower(-0.1);
                    Motor3.setPower(0.1);
                    telemetry.addData("Spike",numOfSpikes);
                    telemetry.update();
                    if(rangeSensor.getDistance(DistanceUnit.CM)<26){
                        numOfSpikes++;
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){
                    Motor4.setPower(0);
                    Motor3.setPower(0); //pause
                    }
                    Timer.reset();
                    while(Timer.seconds()<.5){
                        Motor4.setPower(0.1);
                        Motor3.setPower(-0.1); //inch back
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){
                        Motor2.setPower(-0.1);
                        Motor1.setPower(0.1); //glyph goes in
                    }
                }
        
            }else if(VuMark == "LEFT"){
                  int numOfSpikes = 2;
                while(numOfSpikes < 2){
                    Motor4.setPower(0.1);
                    Motor3.setPower(-0.1);
                    telemetry.addData("Spike",numOfSpikes);
                    telemetry.update();
                    if(rangeSensor.getDistance(DistanceUnit.CM)<26){
                        numOfSpikes++;
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){
                    Motor4.setPower(0);
                    Motor3.setPower(0); //pause
                    }
                    Timer.reset();
                    while(Timer.seconds()<.5){
                        Motor4.setPower(-0.1);
                        Motor3.setPower(0.1); //inch back
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){
                        Motor2.setPower(-0.1);
                        Motor1.setPower(0.1); //glyph goes in
                    }
                }
            }else if(VuMark == "CENTER"){
                int numOfSpikes = 1;
                while(numOfSpikes < 1){
                    Motor4.setPower(-0.1);
                    Motor3.setPower(0.1); //move until spike is detected
                    telemetry.addData("Spike", numOfSpikes);
                    telemetry.update();
                    if(rangeSensor.getDistance(DistanceUnit.CM)<26){
                        numOfSpikes++;
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){
                    Motor4.setPower(0);
                    Motor3.setPower(0); //pause for a bit 
                    }
                    Timer.reset();
                    while(Timer.seconds()<.5){ //seconds needs testing
                        Motor4.setPower(0.1);
                        Motor3.setPower(-0.1); //inch back to center
                    }
                    Timer.reset();
                    while(Timer.seconds()<.3){ //seconds needs testing
                        Motor2.setPower(-0.1);
                        Motor1.setPower(0.1); //glyph goes in
                    }
                }
            }
        

            telemetry.update();
        }
    }
}

