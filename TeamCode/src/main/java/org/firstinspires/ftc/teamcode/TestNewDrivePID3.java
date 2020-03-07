package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

@Autonomous(name = "TestNewDrivePID3 (Blocks to Java)", group = "")
public class TestNewDrivePID3 extends LinearOpMode {

  private DcMotor LeftFront;
  private DcMotor LeftRear;
  private DcMotor RightFront;
  private DcMotor RightRear;
  private DigitalChannel touch1;
  private Servo Gripper;
  private DcMotor LeftIntake;
  private DcMotor RightIntake;
  private DcMotor extension;
  private Servo camera;
  private Servo FoundationLeft;
  private Servo FoundationRight;
  private TfodSkyStone tfodSkyStone;
  private Servo Wrist;
  private DcMotor lift;
  private VuforiaSkyStone vuforiaSkyStone;
  private TouchSensor FoundationTouch;
  private BNO055IMU imu1;

  double inch_velocity;
  ElapsedTime Timer;
  double Steer2;
  double Skystone_Position;
  double Mid_Position;
  double Red_Or_Blue;
  double Angle_Function;
  double Base_Power;
  double Turn_P;
  double Turn_Amount;
  int Strafe_Multiplier;
  double Auto_Selector2;
  double Remaining_Angle;
  double VerticalPlace;
  double Done_Angle;
  double Left_Velocity;
  double DeliverDone;
  long Wait_Time;
  double Total_Error;
  double Right_Velocity;
  double Last_Error;
  double TurnD;
  double Loop_Number;
  double TurnI;
  double Park_Inside;
  double Foundation_or_Depot;
  double Correction;
  double Turn_Power_Percentage;
  double Turn_Power;
  double Motor_Correction;

  /**
   * Describe this function...
   */
  private void Button_Drive(float Final_Raw, double velocity, double Percent_Slow_Point) {
    // Put loop blocks here
    int Final_Position = Math.round(Final_Raw);
    LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + Final_Position);
    LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Final_Position);
    RightFront.setTargetPosition(RightFront.getCurrentPosition() + Final_Position);
    RightRear.setTargetPosition(RightRear.getCurrentPosition() + Final_Position);
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Right_Velocity = 0.5;
    Left_Velocity = 0.5;
    while (opModeIsActive() && (LeftFront.isBusy() || RightFront.isBusy())) {
      LeftFront.setPower(Left_Velocity);
      RightFront.setPower(Right_Velocity);
      LeftRear.setPower(Left_Velocity);
      RightRear.setPower(Right_Velocity);
      if (touch1.getState() == false) {
        Gripper.setPosition(1);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
      }
      telemetry.addData("Power L", Left_Velocity);
      telemetry.addData("Power R", Right_Velocity);
      telemetry.addData("velo", velocity);
      telemetry.addData("Steer", Steer2);
      telemetry.addData("Correction", Correction);
      telemetry.update();
    }
    LeftIntake.setPower(0);
    RightIntake.setPower(0);
    Gripper.setPosition(1);
    telemetry.addData("Done", 1);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Strafe_Intake(float Final_Raw) {
    // Put loop blocks here.
    int Final_Position = Math.round(Final_Raw);
    if (Red_Or_Blue == 1) {
      RightRear.setTargetPosition(RightRear.getCurrentPosition() + Final_Position);
      LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + Final_Position);
      LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    } else {
      RightFront.setTargetPosition(RightFront.getCurrentPosition() + Final_Position);
      LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Final_Position);
      RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    Left_Velocity = 1;
    Right_Velocity = 1;
    LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftIntake.setPower(1);
    RightIntake.setPower(1);
    if (Red_Or_Blue == 1) {
      while (opModeIsActive() && LeftFront.isBusy()) {
        RightRear.setPower(Right_Velocity);
        LeftFront.setPower(Left_Velocity);
        if (touch1.getState() == false) {
          Gripper.setPosition(1);
          LeftIntake.setPower(0);
          RightIntake.setPower(0);
        }
      }
    } else {
      while (opModeIsActive() && RightFront.isBusy()) {
        LeftRear.setPower(Left_Velocity);
        RightFront.setPower(Right_Velocity);
        if (touch1.getState() == false) {
          Gripper.setPosition(1);
          LeftIntake.setPower(0);
          RightIntake.setPower(0);
        }
      }
    }
    Gripper.setPosition(1);
    LeftIntake.setPower(0);
    RightIntake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Strafe_Foundation(float Final_Raw)
      // TODO: Enter the type for argument named Strafe_Percent_Slow_Point
       {
    boolean Is_done_Foundation_Turn;
    int Final_Position = Math.round(Final_Raw);
    // Put loop blocks here.
    if (Red_Or_Blue == 1) {
      RightRear.setTargetPosition(RightRear.getCurrentPosition() - Final_Position);
      LeftFront.setTargetPosition(LeftFront.getCurrentPosition() - Final_Position);
      LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    } else {
      RightFront.setTargetPosition(RightFront.getCurrentPosition() - Final_Position);
      LeftRear.setTargetPosition(LeftRear.getCurrentPosition() - Final_Position);
      RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    DeliverDone = 0;
    if (Red_Or_Blue == 1) {
      while (opModeIsActive() && LeftFront.isBusy()) {
        Left_Velocity = 1;
        Right_Velocity = 1;
        RightRear.setPower(Right_Velocity);
        LeftFront.setPower(Left_Velocity);
        if (DeliverDone == 0) {
          Deliver_Skystone4();
        }
        DeliverDone = 1;
        telemetry.addData("Target", LeftFront.getTargetPosition());
        telemetry.addData("Target", RightRear.getTargetPosition());
        telemetry.addData("Current L", LeftFront.getCurrentPosition());
        telemetry.addData("Current R", RightRear.getCurrentPosition());
        telemetry.addData("In look", "in loop");
        telemetry.update();
      }
    } else {
      while (opModeIsActive() && RightFront.isBusy()) {
        Left_Velocity = 1;
        Right_Velocity = 1;
        LeftRear.setPower(Left_Velocity);
        RightFront.setPower(Right_Velocity);
        if (DeliverDone == 0) {
          Deliver_Skystone4();
        }
        DeliverDone = 1;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Intake(float How_Far) {
    Gripper.setPosition(0);
    LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftIntake.setPower(1);
    RightIntake.setPower(1);
    Button_Drive((float) (16.75 * How_Far), inch_velocity, 0);
    Gripper.setPosition(1);
    LeftIntake.setPower(0);
    RightIntake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Auto_Selector() {
    // TODO: Enter the type for variable named Choice
    String Choice;
    double Done_Auto;
    double Done_Color;
    String Selection;

    Done_Auto = 0;
    Done_Color = 0;
    while (Done_Auto != 1) {
      telemetry.addData("Instructions Color", "X is Blue B is Red");
      telemetry.addData("Press DPad Down For", "Default/Test");
      telemetry.update();
      while (Done_Color != 1) {
        if (gamepad1.dpad_down) {
          Selection = "Default/Test";
          Auto_Selector2 = 1;
          Red_Or_Blue = 0;
          Done_Color = verify(Selection);
          Done_Auto = Done_Color;
        }
        if (gamepad1.x) {
          Red_Or_Blue = 0;
          Selection = "Blue";
          Done_Color = verify(Selection);
        }
        if (gamepad1.b) {
          Red_Or_Blue = 1;
          Selection = "Red";
          Done_Color = verify(Selection);
        }
      }
      while (Done_Auto != 1) {
        telemetry.addData("Dpad Up", "Two Stone Yeet");
        telemetry.addData("DPad Right", "One Stone Foundation Yeet Second Stone");
        telemetry.addData("DPad Left", "Foundation");
        telemetry.addData("DPad Down", "Default/Test");
        telemetry.update();
        if (gamepad1.dpad_up) {
          Auto_Selector2 = 1;
          Selection = "Two Stone Yeet";
          Done_Auto = verify(Selection);
        } else if (gamepad1.dpad_right) {
          Auto_Selector2 = 2;
          Selection = "One Stone Foundation";
          Done_Auto = verify(Selection);
        } else if (gamepad1.dpad_left) {
          Auto_Selector2 = 4;
          Selection = "Foundation";
          Done_Auto = verify(Selection);
        }
      }
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("Left Front");
    LeftRear = hardwareMap.dcMotor.get("Left Rear");
    RightFront = hardwareMap.dcMotor.get("Right Front");
    RightRear = hardwareMap.dcMotor.get("Right Rear");
    touch1 = hardwareMap.digitalChannel.get("touch1");
    Gripper = hardwareMap.servo.get("Gripper");
    LeftIntake = hardwareMap.dcMotor.get("Left Intake");
    RightIntake = hardwareMap.dcMotor.get("Right Intake");
    extension = hardwareMap.dcMotor.get("extension");
    camera = hardwareMap.servo.get("camera");
    FoundationLeft = hardwareMap.servo.get("Foundation Left");
    FoundationRight = hardwareMap.servo.get("FoundationRight");
    tfodSkyStone = new TfodSkyStone();
    Wrist = hardwareMap.servo.get("Wrist");
    lift = hardwareMap.dcMotor.get("lift");
    vuforiaSkyStone = new VuforiaSkyStone();
    FoundationTouch = hardwareMap.touchSensor.get("Foundation Touch");
    imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");

    // Put initialization blocks here.
    Set_Options();
    telemetry.addData("Init Start", "Wait for Start Indication");
    telemetry.update();
    inch_velocity = 1000;
    extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extension.setDirection(DcMotorSimple.Direction.REVERSE);
    camera.setPosition(0.8);
    FoundationLeft.setPosition(0.94);
    FoundationRight.setPosition(0);
    Camera_Init();
    InitIMU();
    Reset_Lift();
    Reset_Motors();
    tfodSkyStone.activate();
    Wrist.setPosition(0.82);
    telemetry.addData("Init Done", "Ready to Press Start");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      if (Auto_Selector2 == 3) {
        inch_velocity = 1500;
        sleep(Wait_Time);
        if (Park_Inside == 0) {
          Drive_Inches(25);
        }
        if (Foundation_or_Depot == 1) {
          Strafe_Inches(-16);
        } else {
          Strafe_Inches(16);
        }
      } else if (Auto_Selector2 == 4) {
        sleep(Wait_Time);
        if (Red_Or_Blue == 1) {
          Strafe_Inches(12);
        } else {
          Strafe_Inches(18);
        }
        inch_velocity = 1000;
        Drive_Inches(29);
        Move_Foundation();
        inch_velocity = 1500;
        if (Red_Or_Blue == 1) {
          Strafe_Inches(-47);
        } else {
          Strafe_Inches(-50);
        }
        Drive_Inches(-3);
      } else {
        if (Red_Or_Blue == 0) {
          // used to be .5
          camera.setPosition(0.47);
          Foundation_Get_Stone();
          Drive_Inches(15);
        } else {
          camera.setPosition(0.55);
          Foundation_Get_Stone();
          Drive_Inches(15);
        }
        Get_Stone(FindSkystone());
      }
    }

    tfodSkyStone.close();
    vuforiaSkyStone.close();
  }

  /**
   * Describe this function...
   */
  private void Set_Options() {
    // Auto Selector 0 = 1 stone yeet park
    // Auto Selector 1 = 2 stone yeet park
    // Auto Selector 2 = 1 stone place move  park
    // Auto Selector 3 = Park Outside
    // Auto Selector 4 = Move Foundation
    // Auto Selector 5 = 2 Stone Place One Park
    // Auto Selector 6 = 2 Stone Place Two Park
    // Foundation Vertical=1 Foundation Horizontal=0
    VerticalPlace = 0;
    Wait_Time = 1;
    // Blue=0 Red=1
    Auto_Selector();
    // Outside=0 Inside=1
    Park_Inside = 1;
    // Depot=0 Foundation=1
    Foundation_or_Depot = 0;
    if (Red_Or_Blue == 0) {
      Strafe_Multiplier = 1;
    } else {
      Strafe_Multiplier = -1;
    }
    Wait_Time = Wait_Time * 1000;
  }

  /**
   * Describe this function...
   */
  private double verify(String Choice) {
    double Done_Path;
    double Verify;

    Done_Path = 0;
    Verify = 0;
    telemetry.addData("You Selected", Choice);
    telemetry.addData("Are You Sure? ", "Press Y For Yes B For No");
    telemetry.update();
    while (Done_Path != 1) {
      if (gamepad1.y) {
        Verify = 1;
        Done_Path = 1;
      }
      if (gamepad1.b) {
        Verify = 0;
        Done_Path = 1;
      }
    }
    return Verify;
  }

  /**
   * Describe this function...
   */
  private void Foundation_Get_Stone() {
    if (Red_Or_Blue == 0) {
      FoundationRight.setPosition(0.5);
    } else {
      FoundationLeft.setPosition(0.35);
    }
  }

  /**
   * Describe this function...
   */
  private void Skystone_Right() {
    inch_velocity = 1500;
    if (Red_Or_Blue == 0) {
      Strafe_Inches(-6);
    } else {
      Strafe_Inches(-21);
    }
    Intake(35);
    Foundation_Up();
    inch_velocity = 1500;
    if (Auto_Selector2 == 5) {
      Drive_Inches(-24.5);
    } else {
      if (Red_Or_Blue == 0) {
        Drive_Inches(-24);
      } else {
        Drive_Inches(-21);
      }
    }
    if (Auto_Selector2 == 1 || Auto_Selector2 == 5) {
      Drive_Inches(55);
      Yeet_Skystone();
      if (Red_Or_Blue == 0) {
        Drive_Inches(-64);
      } else {
        Drive_Inches(-64);
      }
      Angle_Function = TurnDegrees(290);
      if (Red_Or_Blue == 0) {
        Strafe_Inches(12);
      } else {
        Strafe_Inches(11);
      }
      Intake(9);
      inch_velocity = 1500;
      Drive_Inches(-10);
      if (Red_Or_Blue == 0) {
        Angle_Function = TurnDegrees(89);
      } else {
        Angle_Function = TurnDegrees(91);
      }
      if (Red_Or_Blue == 0) {
        Strafe_Inches(15);
      } else {
        Strafe_Inches(12);
      }
      if (Red_Or_Blue == 5) {
      } else {
      }
      Drive_Inches(65);
      Yeet_Skystone();
      Drive_Inches(-10);
    } else if (Auto_Selector2 == 2) {
      inch_velocity = 1750;
      if (Red_Or_Blue == 0) {
        Strafe_Inches(93);
      } else {
        Strafe_Inches(93);
      }
      drive_in((float)(25 * 16.75));
      Foundation_Down();
      sleep(300);
      if (Red_Or_Blue == 0) {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(-34 * 16.75, -20, -20);
      } else {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(34 * 16.75, -20, -20);
      }
      Foundation_Up();
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Strafe_Foundation((float)(35 * 19.5));
      } else {
        Strafe_Foundation(((float)(33 * 19.5)));
      }
      Angle_Function = TurnDegrees(88);
      if (Red_Or_Blue == 0) {
        Drive_Inches(-64);
      } else {
        Drive_Inches(-63);
      }
      sleep(300);
      Angle_Function = TurnDegrees(285);
      if (Red_Or_Blue == 0) {
        Strafe_Inches(24);
      } else {
        Strafe_Inches(14);
      }
      Intake(7);
      if (Red_Or_Blue == 0) {
        Strafe_Foundation((float)(28 * 19.5));
      } else {
        Strafe_Foundation((float)(25 * 19.5));
      }
      Angle_Function = TurnDegrees(90);
      Drive_Inches(60);
      Yeet_Skystone();
      Drive_Inches(-15);
    } else {
      Yeet_Skystone();
      Drive_Inches(-17);
    }
  }

  /**
   * Describe this function...
   */
  private void SKystone_Center() {
    if (Red_Or_Blue == 0) {
      Strafe_Inches(6.5);
    } else {
      Strafe_Inches(-13);
    }
    Intake(35);
    if (Auto_Selector2 == 6) {
      Angle_Function = TurnDegrees(270);
      Drive_Inches(-80);
      Angle_Function = TurnDegrees(0);
      Drive_Inches(4);
      Deliver_Skystone1();
    } else {
      if (Auto_Selector2 == 5) {
        Drive_Inches(-27);
      } else if (Auto_Selector2 == 1) {
        // -22 before strafe
        Drive_Inches(-30);
      } else {
        if (Red_Or_Blue == 0) {
          Drive_Inches(-25);
        } else {
          Drive_Inches(-23);
        }
      }
      // Disabled for attempt to strafe
    }
    if (Auto_Selector2 == 1 || Auto_Selector2 == 5) {
      if (Auto_Selector2 == 5) {
        if (VerticalPlace == 1) {
          Drive_Inches(18.5);
          Deliver_Skystone1();
          Drive_Inches(-18.5);
          sleep(750);
        } else {
          if (Red_Or_Blue == 0) {
            Drive_Inches(36);
            Deliver_Skystone1();
            Drive_Inches(-34.5);
          } else {
            Drive_Inches(34);
            Deliver_Skystone1();
            Drive_Inches(-32.5);
          }
        }
      } else {
        Drive_Inches(47);
        Yeet_Skystone();
      }
      Drive_Inches(-65);
      Angle_Function = TurnDegrees(0);
      Foundation_Get_Stone();
      if (Red_Or_Blue == 0) {
        Intake(35);
      } else {
        Intake(35);
      }
      Foundation_Up();
      inch_velocity = 1500;
      if (Red_Or_Blue == 0) {
        Drive_Inches(-31);
      } else {
        Drive_Inches(-31);
      }
      if (Red_Or_Blue == 0) {
        Angle_Function = TurnDegrees(92);
      } else {
        Angle_Function = TurnDegrees(92);
      }
      Drive_Inches(73);
      Yeet_Skystone();
      Drive_Inches(-15);
    } else if (Auto_Selector2 == 2) {
      Foundation_Up();
      if (Red_Or_Blue == 0) {
        Strafe_Inches(81);
      } else {
        Strafe_Inches(84);
      }
      drive_in((float)(22 * 16.75));
      Foundation_Down();
      sleep(250);
      if (Red_Or_Blue == 0) {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(-32 * 16.75, -20, -20);
      } else {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(22 * 16.75, -20, -20);
      }
      Foundation_Up();
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Strafe_Foundation((float)(33 * 19.5));
      } else {
        Strafe_Foundation((float)(29 * 19.5));
      }
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Drive_Inches(-71);
      } else {
        Drive_Inches(-71);
      }
      Foundation_Get_Stone();
      Angle_Function = TurnDegrees(355);
      Intake(35);
      if (Red_Or_Blue == 0) {
        Drive_Inches(-23);
      } else {
        Drive_Inches(-27);
      }
      Foundation_Up();
      Angle_Function = TurnDegrees(90);
      Drive_Inches(68);
      Yeet_Skystone();
      Drive_Inches(-17);
    } else {
      Yeet_Skystone();
      Drive_Inches(-17);
    }
  }

  /**
   * Describe this function...
   */
  private void Reset_Motors() {
    double VelocityP;
    double p;
    double D;
    PIDFCoefficients PIDF;

    VelocityP = 7;
    p = 1.5;
    D = 0;
    ((DcMotorEx) LeftFront).setPositionPIDFCoefficients(p);
    ((DcMotorEx) RightFront).setPositionPIDFCoefficients(p);
    ((DcMotorEx) LeftRear).setPositionPIDFCoefficients(p);
    ((DcMotorEx) RightRear).setPositionPIDFCoefficients(p);
    ((DcMotorEx) LeftFront).setTargetPositionTolerance(50);
    ((DcMotorEx) RightFront).setTargetPositionTolerance(50);
    ((DcMotorEx) LeftRear).setTargetPositionTolerance(50);
    ((DcMotorEx) RightRear).setTargetPositionTolerance(50);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  /**
   * Describe this function...
   */
  private void Move_Foundation() {
    FoundationLeft.setPosition(0.52);
    FoundationRight.setPosition(0.4);
    sleep(2000);
    if (Red_Or_Blue == 0) {
      Fast_Drive((float)(-35 * 16.75));
    } else {
      Fast_Drive((float)(-33.5 * 16.75));
    }
    FoundationLeft.setPosition(0.39);
    FoundationRight.setPosition(0.9);
  }

  /**
   * Describe this function...
   */
  private void Foundation_Up() {
    FoundationRight.setPosition(0.89);
    FoundationLeft.setPosition(0.07);
  }

  /**
   * Describe this function...
   */
  private void Foundation_Down() {
    FoundationRight.setPosition(0.4);
    FoundationLeft.setPosition(0.52);
  }

  /**
   * Describe this function...
   */
  private void Skystone_Left() {
    inch_velocity = 1500;
    if (Red_Or_Blue == 0) {
      Strafe_Inches(14.5);
    } else {
      Strafe_Inches(-5);
    }
    Intake(35);
    Foundation_Up();
    inch_velocity = 1500;
    // -22 before strafe
    if (Auto_Selector2 == 5) {
      Drive_Inches(-22);
    } else {
      Drive_Inches(-22);
    }
    if (Auto_Selector2 == 1 || Auto_Selector2 == 5) {
      if (Auto_Selector2 == 5) {
        if (VerticalPlace == 1) {
          Drive_Inches(18.5);
          Deliver_Skystone1();
          Drive_Inches(-18.5);
          sleep(750);
        } else {
          if (Red_Or_Blue == 0) {
            Drive_Inches(74);
            Deliver_Skystone1();
            Drive_Inches(-33);
          } else {
            Drive_Inches(74);
            Deliver_Skystone1();
            Drive_Inches(-32.5);
          }
        }
      } else {
        Drive_Inches(43);
        Yeet_Skystone();
      }
      Drive_Inches(-62);
      Angle_Function = TurnDegrees(0);
      Foundation_Get_Stone();
      if (Red_Or_Blue == 0) {
        Intake(40);
      } else {
        Intake(40);
      }
      Foundation_Up();
      inch_velocity = 1500;
      if (Red_Or_Blue == 0) {
        Drive_Inches(-35);
      } else {
        Drive_Inches(-37);
      }
      Angle_Function = TurnDegrees(92);
      Drive_Inches(60);
      Yeet_Skystone();
      Drive_Inches(-7);
    } else if (Auto_Selector2 == 2) {
      Foundation_Up();
      if (Red_Or_Blue == 0) {
        Strafe_Inches(77);
      } else {
        Strafe_Inches(75);
      }
      drive_in((float)(22 * 16.75));
      Foundation_Down();
      sleep(300);
      if (Red_Or_Blue == 0) {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(-34 * 16.75, -20, -20);
      } else {
        Fast_Drive((float)(-27 * 16.75));
        Foundation_Turn(34 * 16.75, -20, -20);
      }
      Foundation_Up();
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Strafe_Foundation((float)(35 * 19.5));
      } else {
        Strafe_Foundation((float)(34 * 19.5));
      }
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Drive_Inches(-57);
      } else {
        Drive_Inches(-57);
      }
      Foundation_Get_Stone();
      Angle_Function = TurnDegrees(355);
      Intake(30);
      if (Red_Or_Blue == 0) {
        Drive_Inches(-22);
      } else {
        Drive_Inches(-26);
      }
      Foundation_Up();
      Angle_Function = TurnDegrees(90);
      if (Red_Or_Blue == 0) {
        Drive_Inches(58);
      } else {
        Drive_Inches(62);
      }
      Yeet_Skystone();
      Drive_Inches(-17);
    } else {
      Yeet_Skystone();
      Drive_Inches(-17);
    }
  }

  /**
   * Describe this function...
   */
  private void Button_Stop() {
    Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    while (!(touch1.getState() == false || Timer.milliseconds() > 4000)) {
    }
    Gripper.setPosition(1);
    LeftIntake.setPower(0);
    RightIntake.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Get_Stone(double position) {
    tfodSkyStone.deactivate();
    if (Red_Or_Blue == 0) {
      if (Skystone_Position == 0) {
        Skystone_Left();
      } else if (Skystone_Position == 2) {
        Skystone_Right();
      } else {
        SKystone_Center();
      }
    } else {
      if (Skystone_Position == 0) {
        SKystone_Center();
      } else if (Skystone_Position == 2) {
        Skystone_Right();
      } else {
        Skystone_Left();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Reset_Lift() {
    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Gripper.setPosition(0);
  }

  /**
   * Describe this function...
   */
  private void Run_Motor_fast() {
    LeftFront.setPower(0.5);
    RightFront.setPower(0.5);
    LeftRear.setPower(0.5);
    RightRear.setPower(0.5);
    sleep(250);
    LeftFront.setPower(0);
    RightFront.setPower(0);
    LeftRear.setPower(0);
    RightRear.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Deliver_Skystone1() {
    Gripper.setPosition(1);
    lift.setTargetPosition(1300);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
    sleep(200);
  }

  /**
   * Describe this function...
   */
  private void Deliver_Skystone2() {
    extension.setTargetPosition(1300);
    extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extension.setPower(1);
    sleep(500);
    lift.setTargetPosition(200);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
    sleep(100);
    Gripper.setPosition(0);
    sleep(300);
    lift.setTargetPosition(1300);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
  }

  /**
   * Describe this function...
   */
  private void Foundation_Turn(double Final_Position, double velocity, double Percent_Slow_Point) {
    double Desired_Angle;

    Desired_Angle = 80;
    // NO NEGATIVE DEGREES - Desired Angle must be 0-360
    if (Red_Or_Blue == 1) {
      Desired_Angle = 360 - Desired_Angle;
      if (Desired_Angle == 360) {
        Desired_Angle = 0;
      }
    }
    Reset_Motor_Using_Encoder();
    Base_Power = 1;
    Turn_P = 0.023;
    Turn_Amount = Desired_Angle - GetDegrees();
    Remaining_Angle = Turn_Amount;
    Done_Angle = Math.abs(Remaining_Angle);
    Total_Error = 0;
    Last_Error = 0;
    TurnD = 0.055;
    TurnI = 0.0003;
    if (Math.abs(Turn_Amount) > 2) {
      if (Math.abs(Remaining_Angle) > 180) {
        if (Remaining_Angle > 0) {
          Remaining_Angle = Remaining_Angle - 360;
        } else {
          Remaining_Angle = Remaining_Angle + 360;
        }
      }
      while (opModeIsActive() && Done_Angle > 5) {
        if (Remaining_Angle / Total_Error > 0) {
          if (Math.abs(Remaining_Angle + Total_Error) * TurnI < 1 && Math.abs(Remaining_Angle + Total_Error) * TurnI > -1) {
            Total_Error += Remaining_Angle;
          }
        } else {
          Total_Error = 0;
        }
        Motor_Correction = Remaining_Angle * Turn_P + Total_Error * TurnI + TurnD * (Remaining_Angle - Last_Error);
        if (Red_Or_Blue == 1) {
          Turn_Power = Base_Power * Motor_Correction;
          Turn_Power = Math.min(Math.max(Turn_Power, -1), 1);
        } else {
          Turn_Power = Base_Power * Motor_Correction;
          Turn_Power = Math.min(Math.max(Turn_Power, -1), 1);
        }
        Last_Error = Remaining_Angle;
        LeftFront.setPower(-Turn_Power);
        LeftRear.setPower(-Turn_Power);
        RightFront.setPower(Turn_Power);
        RightRear.setPower(Turn_Power);
        telemetry.addData("Remaining Angle", Remaining_Angle);
        telemetry.addData("Total Error", Total_Error);
        telemetry.addData("Motor Correction", Motor_Correction);
        telemetry.addData("TurnP", Turn_P);
        telemetry.addData("Turn Power", Turn_Power);
        telemetry.addData("Out of loop", "no");
        Loop_Number += 1;
        telemetry.addData("Loop Number", Loop_Number);
        sleep(25);
        Remaining_Angle = Desired_Angle - GetDegrees();
        if (Math.abs(Remaining_Angle) > 180) {
          if (Remaining_Angle > 0) {
            Remaining_Angle = Remaining_Angle - 360;
          } else {
            Remaining_Angle = Remaining_Angle + 360;
          }
        }
        Done_Angle = Math.abs(Remaining_Angle);
        telemetry.addData("Remaining Angle 2", Remaining_Angle);
        telemetry.addData("Done Angle", Done_Angle);
        telemetry.update();
      }
      RightRear.setPower(0);
      LeftFront.setPower(0);
      LeftRear.setPower(0);
      RightFront.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Deliver_Skystone4() {
    Gripper.setPosition(1);
    extension.setTargetPosition(1);
    extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extension.setPower(0.75);
    Gripper.setPosition(1);
    while (opModeIsActive() && extension.getCurrentPosition() > 75) {
    }
    lift.setTargetPosition(1);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
  }

  /**
   * Describe this function...
   */
  private void Strafe(double Final_Raw) {
    // Put loop blocks here.
    int Final_Position = Math.round((float)(Final_Raw));
    LeftFront.setTargetPosition(LeftFront.getCurrentPosition() - Final_Position * Strafe_Multiplier);
    LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Final_Position * Strafe_Multiplier);
    RightFront.setTargetPosition(RightFront.getCurrentPosition() + Final_Position * Strafe_Multiplier);
    RightRear.setTargetPosition(RightRear.getCurrentPosition() - Final_Position * Strafe_Multiplier);
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Left_Velocity = 1;
    Right_Velocity = 1;
    while (opModeIsActive() && LeftFront.isBusy() && RightFront.isBusy() && LeftRear.isBusy() && RightRear.isBusy()) {
      LeftFront.setPower(Left_Velocity);
      LeftRear.setPower(Left_Velocity);
      RightFront.setPower(Right_Velocity);
      RightRear.setPower(Right_Velocity);
    }
    LeftFront.setPower(0);
    LeftRear.setPower(0);
    RightFront.setPower(0);
    RightRear.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Camera_Init() {
    vuforiaSkyStone.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        true, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        false); // useCompetitionFieldTargetLocations
    tfodSkyStone.initialize(vuforiaSkyStone, 0.6F, true, true);
  }

  /**
   * Describe this function...
   */
  private void Foundation_Turn3(
      // TODO: Enter the type for argument named Final_Position
      double Final_Position,
      // TODO: Enter the type for argument named velocity
      double velocity,
      // TODO: Enter the type for argument named Percent_Slow_Point
      double Percent_Slow_Point) {
    // Put loop blocks here.
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    Steer2 = GetDegrees();
    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    while (opModeIsActive() && Math.abs(Steer2 - 90) >= 5) {
      if (Steer2 <= 90) {
        Left_Velocity = 1;
        Right_Velocity = -1;
      } else {
        Left_Velocity = -1;
        Right_Velocity = 1;
      }
      LeftFront.setPower(Left_Velocity);
      RightFront.setPower(Right_Velocity);
      LeftRear.setPower(Left_Velocity);
      RightRear.setPower(Right_Velocity);
      Steer2 = GetDegrees();
      telemetry.addData("Power L", Left_Velocity);
      telemetry.addData("Power R", Right_Velocity);
      telemetry.update();
    }
  }


  /**
   * Describe this function...
   */
  private void AutoDrive(float Final_Raw) {
    // Put loop blocks here.
    int Final_Position = Math.round((float)(Final_Raw));
    LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + Final_Position);
    LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Final_Position);
    RightFront.setTargetPosition(RightFront.getCurrentPosition() + Final_Position);
    RightRear.setTargetPosition(RightRear.getCurrentPosition() + Final_Position);
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Left_Velocity = 1;
    Right_Velocity = 1;
    while (opModeIsActive() && (LeftFront.isBusy() || RightFront.isBusy())) {
      LeftFront.setPower(Left_Velocity);
      RightFront.setPower(Right_Velocity);
      LeftRear.setPower(Left_Velocity);
      RightRear.setPower(Right_Velocity);
    }
    LeftFront.setPower(0);
    LeftRear.setPower(0);
    RightFront.setPower(0);
    RightRear.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void drive_in(float Final_Raw) {
    int Final_Position = Math.round((float)(Final_Raw));
    // Put loop blocks here.
    LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + Final_Position);
    LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Final_Position);
    RightFront.setTargetPosition(RightFront.getCurrentPosition() + Final_Position);
    RightRear.setTargetPosition(RightRear.getCurrentPosition() + Final_Position);
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    Loop_Number = 0;
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    DeliverDone = 0;
    Left_Velocity = 1;
    Right_Velocity = 1;
    while (opModeIsActive() && (LeftFront.isBusy() || RightFront.isBusy()) && !FoundationTouch.isPressed()) {
      if (!FoundationTouch.isPressed()) {
        LeftFront.setPower(Left_Velocity);
        RightFront.setPower(Right_Velocity);
        LeftRear.setPower(Left_Velocity);
        RightRear.setPower(Right_Velocity);
      } else {
        LeftFront.setPower(0);
        LeftRear.setPower(0);
        RightFront.setPower(0);
        RightRear.setPower(0);
      }
      if (DeliverDone == 0) {
        Deliver_Skystone1();
        DeliverDone = 1;
      }
    }
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setPower(0);
    LeftRear.setPower(0);
    RightFront.setPower(0);
    RightRear.setPower(0);
  }

  /**
   * Describe this function...
   */

  private void Strafe_Inches(double Inches) {
    Strafe(19.5 * Inches);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  /**
   * Describe this function...
   */
  private double Get_Error(
      // TODO: Enter the type for argument named Target_Angle
      double Target_Angle) {
    double Robot_Error;

    Robot_Error = Target_Angle - GetDegrees();
    if (Math.abs(Robot_Error) > 180) {
      if (Robot_Error > 0) {
        Robot_Error = Robot_Error - 360;
      } else {
        Robot_Error = Robot_Error + 360;
      }
    }
    telemetry.addData("Get Degrees", GetDegrees());
    return Robot_Error;
  }

  /**
   * Describe this function...
   */
  private boolean Is_Lift_High_Enough() {
    boolean High_Enough;

    if (lift.getCurrentPosition() >= 730) {
      High_Enough = true;
    } else {
      High_Enough = false;
    }
    return High_Enough;
  }

  /**
   * Describe this function...
   */
  private void Drive_Inches(double Inches) {
    AutoDrive((float)(16.75 * Inches));
  }

  /**
   * Describe this function...
   */
  private void InitIMU() {
    BNO055IMU.Parameters imuParameters;

    // Create new IMU Parameters object.
    imuParameters = new BNO055IMU.Parameters();
    // Use degrees as angle unit.
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // Express acceleration as m/s^2.
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // Disable logging.
    imuParameters.loggingEnabled = false;
    // Initialize IMU.
    imu1.initialize(imuParameters);
    // Prompt user to press start buton.
  }

  /**
   * Describe this function...
   */
  private double FindSkystone() {
    List<Recognition> recognitions;
    double SkystoneX;
    double Block1X;
    double Block2X;

    if (opModeIsActive()) {
      // Put run blocks here.
      Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
      Skystone_Position = -1;
      while (!(Skystone_Position > -1 || Timer.milliseconds() > 2000)) {
        // Put loop blocks here.
        recognitions = tfodSkyStone.getRecognitions();
        if (recognitions.size() > 1) {
          SkystoneX = -1;
          Block1X = -1;
          Block2X = -1;
          for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals("Skystone")) {
              SkystoneX = recognition.getLeft();
            } else if (Block1X == -1) {
              Block1X = recognition.getLeft();
            } else {
              Block2X = recognition.getLeft();
            }
          }
          // Make sure we found one gold mineral and two silver minerals.
          if (SkystoneX != -1 && Block1X != -1) {
            if (SkystoneX < Block1X) {
              Skystone_Position = 0;
            } else if (SkystoneX > Block1X) {
              Skystone_Position = 1;
            } else {
              Skystone_Position = 2;
            }
          } else if (Block2X != -1 && Block1X != -1) {
            Skystone_Position = 2;
          }
        }
      }
      telemetry.addData("Skystone Position", Skystone_Position);
    }
    return Skystone_Position;
  }

  /**
   * Describe this function...
   */
  private void Fast_PID() {
    ((DcMotorEx) LeftFront).setVelocityPIDFCoefficients(100, 0, 0, 0);
    ((DcMotorEx) LeftFront).setPositionPIDFCoefficients(12);
    ((DcMotorEx) RightFront).setVelocityPIDFCoefficients(100, 0, 0, 0);
    ((DcMotorEx) RightFront).setPositionPIDFCoefficients(12);
    ((DcMotorEx) LeftRear).setVelocityPIDFCoefficients(100, 0, 0, 0);
    ((DcMotorEx) LeftRear).setPositionPIDFCoefficients(12);
    ((DcMotorEx) RightRear).setVelocityPIDFCoefficients(1000, 0, 0, 0);
    ((DcMotorEx) RightRear).setPositionPIDFCoefficients(12);
  }

  /**
   * Describe this function...
   */
  private double GetDegrees() {
    Orientation angles;
    float GetAngle;

    angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    GetAngle = angles.firstAngle;
    return (GetAngle + 360) % 360;
  }

  /**
   * Describe this function...
   */
  private void Reset_Motor_Using_Encoder() {
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void Fast_Drive(float Fast_Raw) {
    ElapsedTime DriveTime;
    double Velocity_Fast;
    double VeloNotFast;
    int Fast_Position = Math.round((float)(Fast_Raw));

    DriveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // Put loop blocks here.
    LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + Fast_Position);
    LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + Fast_Position);
    RightFront.setTargetPosition(RightFront.getCurrentPosition() + Fast_Position);
    RightRear.setTargetPosition(RightRear.getCurrentPosition() + Fast_Position);
    telemetry.addData("Target", LeftFront.getTargetPosition());
    telemetry.addData("Target", RightFront.getTargetPosition());
    telemetry.addData("Target", LeftRear.getTargetPosition());
    telemetry.addData("Target", RightRear.getTargetPosition());
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Velocity_Fast = 1;
    VeloNotFast = 1;
    DriveTime.reset();
    DeliverDone = 0;
    while (opModeIsActive() && (LeftFront.isBusy() || RightFront.isBusy()) && DriveTime.milliseconds() < 3500) {
      if (Red_Or_Blue == 0) {
        LeftFront.setPower(Velocity_Fast);
        RightFront.setPower(VeloNotFast);
        LeftRear.setPower(Velocity_Fast);
        RightRear.setPower(VeloNotFast);
      } else {
        LeftFront.setPower(VeloNotFast);
        RightFront.setPower(Velocity_Fast);
        LeftRear.setPower(VeloNotFast);
        RightRear.setPower(Velocity_Fast);
      }
      if (DeliverDone == 0) {
        Deliver_Skystone2();
      }
      DeliverDone = 1;
    }
    Velocity_Fast = 0;
    LeftFront.setPower(Velocity_Fast);
    RightFront.setPower(Velocity_Fast);
    LeftRear.setPower(Velocity_Fast);
    RightRear.setPower(Velocity_Fast);
    telemetry.addData("Power L", LeftFront.getPower());
    telemetry.addData("Power R", RightFront.getPower());
    telemetry.addData("Done", 1);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void DrivePID(double Final_Position, double velocity, double Percent_Slow_Point) {
    double Drive_P;
    double Drive_I;
    double Drive_D;
    double Tolerance;
    double Left_Target;
    double Right_Target;
    double Left_Total_Remaining;
    double Right_Total_Remaining;
    double Left_Last_Remaining;
    double Right_Last_Remaining;
    double Left_Remaining;
    double Right_Remaining;

    Drive_P = 0.01;
    Drive_I = 0;
    Drive_D = 0;
    Tolerance = 10;
    Base_Power = 1;
    // Put loop blocks here.
    Left_Target = (LeftFront.getCurrentPosition() + Final_Position + LeftRear.getCurrentPosition() + Final_Position) / 2;
    Right_Target = (RightFront.getCurrentPosition() + Final_Position + RightRear.getCurrentPosition() + Final_Position) / 2;
    telemetry.addData("Left Target", Left_Target);
    telemetry.addData("Right Target", Right_Target);
    Left_Total_Remaining = 0;
    Right_Total_Remaining = 0;
    Left_Last_Remaining = 0;
    Right_Last_Remaining = 0;
    Left_Remaining = Left_Target - (LeftFront.getCurrentPosition() + LeftRear.getCurrentPosition()) / 2;
    Right_Remaining = Right_Target - (RightFront.getCurrentPosition() + RightRear.getCurrentPosition()) / 2;
    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    while (opModeIsActive() && (Math.abs(Left_Remaining) > Tolerance || Math.abs(Right_Remaining) > Tolerance)) {
      Left_Velocity = Left_Remaining * Drive_P + Left_Total_Remaining * Drive_I + Drive_D * (Left_Remaining - Left_Last_Remaining);
      Left_Last_Remaining = Left_Remaining;
      Right_Velocity = Right_Remaining * Drive_P + Right_Total_Remaining * Drive_I + Drive_D * (Right_Remaining - Right_Last_Remaining);
      Right_Last_Remaining = Right_Remaining;
      Right_Velocity = Math.min(Math.max(Right_Velocity, -1), 1);
      Left_Velocity = Math.min(Math.max(Left_Velocity, -1), 1);
      LeftFront.setPower(Left_Velocity);
      RightFront.setPower(Right_Velocity);
      LeftRear.setPower(Left_Velocity);
      RightRear.setPower(Right_Velocity);
      if (Math.abs(Left_Remaining + Left_Total_Remaining) * Drive_I < 1 && Math.abs(Left_Remaining + Left_Total_Remaining) * Drive_I > -1) {
        Left_Total_Remaining += Left_Remaining;
      }
      if (Math.abs(Right_Remaining + Right_Total_Remaining) * Drive_I < 1 && Math.abs(Right_Remaining + Right_Total_Remaining) * Drive_I > -1) {
        Right_Total_Remaining += Right_Remaining;
      }
      Left_Remaining = Left_Target - (LeftFront.getCurrentPosition() + LeftRear.getCurrentPosition()) / 2;
      Right_Remaining = Right_Remaining - (RightFront.getCurrentPosition() + RightRear.getCurrentPosition()) / 2;
      telemetry.addData("Power L", Left_Velocity);
      telemetry.addData("Power R", Right_Velocity);
      telemetry.addData("Target L", Left_Target);
      telemetry.addData("Target R", Right_Target);
      telemetry.addData("Current L", Left_Remaining);
      telemetry.addData("Current R", Right_Remaining);
      telemetry.update();
    }
    RightRear.setPower(0);
    LeftFront.setPower(0);
    LeftRear.setPower(0);
    RightFront.setPower(0);
  }

  /**
   * Describe this function...
   */
  private double TurnDegrees(double Desired_Angle) {
    // TODO: Enter the type for variable named Percent_Slow_Point
    double Percent_Slow_Point;

    // NO NEGATIVE DEGREES - Desired Angle must be 0-360
    if (Red_Or_Blue == 1) {
      Desired_Angle = 360 - Desired_Angle;
      if (Desired_Angle == 360) {
        Desired_Angle = 0;
      }
    }
    Reset_Motor_Using_Encoder();
    Base_Power = 1;
    Turn_P = 0.022;
    sleep(30);
    Turn_Amount = Desired_Angle - GetDegrees();
    Remaining_Angle = Turn_Amount;
    Done_Angle = Math.abs(Remaining_Angle);
    Total_Error = 0;
    Last_Error = 0;
    TurnD = 0.04;
    TurnI = 0.001;
    if (Math.abs(Remaining_Angle) > 180) {
      if (Remaining_Angle > 0) {
        Remaining_Angle = Remaining_Angle - 360;
      } else {
        Remaining_Angle = Remaining_Angle + 360;
      }
    }
    Loop_Number = 0;
    while (opModeIsActive() && Done_Angle > 1.2) {
      if (Remaining_Angle / Total_Error > 0) {
        if (Math.abs(Remaining_Angle + Total_Error) * TurnI < 1 && Math.abs(Remaining_Angle + Total_Error) * TurnI > -1) {
          Total_Error += Remaining_Angle;
        }
      } else {
        Total_Error = 0;
      }
      Motor_Correction = Remaining_Angle * Turn_P + Total_Error * TurnI + TurnD * (Remaining_Angle - Last_Error);
      Turn_Power = Base_Power * Motor_Correction;
      Last_Error = Remaining_Angle;
      Turn_Power = Math.min(Math.max(Turn_Power, -1), 1);
      LeftFront.setPower(-Turn_Power);
      LeftRear.setPower(-Turn_Power);
      RightFront.setPower(Turn_Power);
      RightRear.setPower(Turn_Power);
      Loop_Number += 1;
      sleep(30);
      Remaining_Angle = Desired_Angle - GetDegrees();
      if (Math.abs(Remaining_Angle) > 180) {
        if (Remaining_Angle > 0) {
          Remaining_Angle = Remaining_Angle - 360;
        } else {
          Remaining_Angle = Remaining_Angle + 360;
        }
      }
      Done_Angle = Math.abs(Remaining_Angle);
    }
    RightRear.setPower(0);
    LeftFront.setPower(0);
    LeftRear.setPower(0);
    RightFront.setPower(0);
    return 1;
  }

  /**
   * Describe this function...
   */
  private void Yeet_Skystone() {
    Gripper.setPosition(0);
    sleep(200);
    LeftIntake.setPower(-0.8);
    RightIntake.setPower(-0.8);
    Drive_Inches(-5);
    LeftIntake.setPower(0);
    RightIntake.setPower(0);
    lift.setTargetPosition(0);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
  }
}
