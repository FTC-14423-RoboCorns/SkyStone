package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FinalTeleOp (Blocks to Java)", group = "")
public class FinalTeleOp extends LinearOpMode {

  private DcMotor LeftFront;
  private DcMotor RightFront;
  private DcMotor RightRear;
  private DcMotor LeftRear;
  private CRServo Tape;
  private Servo FoundationLeft;
  private Servo FoundationRight;
  private Servo Gripper;
  private DcMotor LeftIntake;
  private DcMotor RightIntake;
  private DigitalChannel touch1;
  private RevBlinkinLedDriver LitLED;
  private Servo Capstone;
  private DcMotor lift;
  private DcMotor extension;
  private Servo camera;
  private Servo Wrist;

  double In_Retention_Loop;
  int Level_Ticks;
  double Speed_Multiplier;
  double Vertical;
  RevBlinkinLedDriver.BlinkinPattern LEDColor;
  ElapsedTime Led_Wait;
  double Level_Count;
  double Horizontal;
  double Cappstone_Down;
  int Lift_Position;
  int Level_Drop;
  double Pivot;

  /**
   * Describe this function...
   */
  private void Strafe() {
    ElapsedTime Timer;

    // Put loop blocks here.
    LeftFront.setPower(1);
    RightFront.setPower(1);
    RightRear.setPower(-1);
    LeftRear.setPower(-1);
    Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    while (!(Timer.milliseconds() > 500)) {
    }
  }

  /**
   * Describe this function...
   */
  private void YEEEEEET() {
    while (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
      if (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
        while (gamepad1.left_trigger > 0.3) {
          Tape.setPower(-gamepad1.left_trigger);
          Drive();
        }
        while (gamepad1.right_trigger > 0.3) {
          Tape.setPower(gamepad1.right_trigger);
          Drive();
        }
      } else {
        Tape.setPower(0);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Drive() {
    if (gamepad2.a) {
      Speed_Multiplier = 0.5;
      Ticks_Per_Color();
      Lift_Early();
    }
    Set_Speed();
    RightFront.setPower(-Pivot + (Vertical - Horizontal));
    LeftFront.setPower(Pivot + Vertical + Horizontal);
    RightRear.setPower(-Pivot + Vertical + Horizontal);
    LeftRear.setPower(Pivot + (Vertical - Horizontal));
  }

  /**
   * Describe this function...
   */
  private void Set_Speed() {
    if (gamepad1.right_bumper) {
      Speed_Multiplier = 0.55;
      sleep(20);
    } else if (gamepad1.left_bumper) {
      Speed_Multiplier = 0.7;
      sleep(20);
    }
    Vertical = -(gamepad1.left_stick_y * Speed_Multiplier);
    Horizontal = gamepad1.left_stick_x * Speed_Multiplier;
    if (0.3 == Speed_Multiplier) {
      Pivot = gamepad1.right_stick_x * (0.1 + Speed_Multiplier);
    } else {
      Pivot = gamepad1.right_stick_x * Speed_Multiplier;
    }
  }

  /**
   * Describe this function...
   */
  private void Intake() {
    if (gamepad1.a) {
      FoundationLeft.setPosition(0.94);
      FoundationRight.setPosition(0);
      Speed_Multiplier = 0.3;
      Ticks_Per_Color();
      Gripper.setPosition(0);
      LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
      LeftIntake.setPower(1);
      RightIntake.setPower(1);
      while (!(touch1.getState() == false || gamepad1.x)) {
        Drive();
        if (gamepad1.y) {
          LeftIntake.setPower(-1);
          RightIntake.setPower(-1);
        }
      }
      Speed_Multiplier = 0.7;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
      LeftIntake.setPower(0);
      RightIntake.setPower(0);
      Gripper.setPosition(0.92);
      sleep(200);
    } else if (gamepad1.y) {
      Gripper.setPosition(0);
      while (gamepad1.y) {
        Drive();
        LeftIntake.setPower(-1);
        RightIntake.setPower(-1);
      }
    } else {
      LeftIntake.setPower(0);
      RightIntake.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Flash_LED() {
    Led_Wait.reset();
    for (int count = 0; count < Level_Count; count++) {
      while (Led_Wait.milliseconds() <= 250) {
        LitLED.setPattern(LEDColor);
      }
      while (Led_Wait.milliseconds() <= 500 && Led_Wait.milliseconds() > 250) {
        LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
      }
    }
    while (Led_Wait.milliseconds() <= 500 && Led_Wait.milliseconds() <= 1500) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("Left Front");
    RightFront = hardwareMap.dcMotor.get("Right Front");
    RightRear = hardwareMap.dcMotor.get("Right Rear");
    LeftRear = hardwareMap.dcMotor.get("Left Rear");
    Tape = hardwareMap.crservo.get("Tape");
    FoundationLeft = hardwareMap.servo.get("Foundation Left");
    FoundationRight = hardwareMap.servo.get("FoundationRight");
    Gripper = hardwareMap.servo.get("Gripper");
    LeftIntake = hardwareMap.dcMotor.get("Left Intake");
    RightIntake = hardwareMap.dcMotor.get("Right Intake");
    touch1 = hardwareMap.digitalChannel.get("touch1");
    LitLED = hardwareMap.get(RevBlinkinLedDriver.class, "LitLED");
    Capstone = hardwareMap.servo.get("Capstone");
    lift = hardwareMap.dcMotor.get("lift");
    extension = hardwareMap.dcMotor.get("extension");
    camera = hardwareMap.servo.get("camera");
    Wrist = hardwareMap.servo.get("Wrist");

    In_Retention_Loop = 0;
    Level_Count = 1;
    Cappstone_Down = 0;
    Speed_Multiplier = 0.6;
    Led_Wait = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Motor_Reset();
    Set_Liift_PID();
    telemetry.addData("Level Count", Level_Count);
    telemetry.addData("Ready to Start", "Ready to Start");
    LEDColor = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;
    telemetry.update();
    waitForStart();
    Capstone.setPosition(0.86);
    Gripper.setPosition(0);
    FoundationRight.setPosition(0.9);
    FoundationLeft.setPosition(0.07);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        Drive();
        Manual_Retention();
        Intake();
        Lift_Level();
        Drop_Capstone();
        Extending_Arm();
        Foundation();
        Capstone2();
        YEEEEEET();
        telemetry.addData("Level Count", Level_Count);
        telemetry.addData("Level Ticks", Level_Ticks);
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Retention", In_Retention_Loop);
        telemetry.addData("Lift Target ", lift.getTargetPosition());
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Set_Liift_PID() {
  }

  /**
   * Describe this function...
   */
  private void Wrist_Manual() {
    double Wrist_Position;

  }

  /**
   * Describe this function...
   */
  private void Retention() {
    double Arm_Retention;

    if (Level_Count > 1) {
      extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      extension.setPower(0.1);
      sleep(50 * Level_Count);
      extension.setPower(0);
      extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      In_Retention_Loop = 1;
      Arm_Retention = 0;
    }
  }

  /**
   * Describe this function...
   */
  private void Manual_Retention() {
    if (gamepad2.right_trigger > 0.1) {
      extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      extension.setPower(0.2);
      sleep(25 * gamepad2.right_trigger);
      extension.setPower(0);
      extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
  }

  /**
   * Describe this function...
   */
  private boolean Is_Lift_High_Enough() {
    boolean High_Enough;

    if (lift.getCurrentPosition() >= 730) {
      High_Enough = 1;
    } else {
      High_Enough = 0;
    }
    return High_Enough;
  }

  /**
   * Describe this function...
   */
  private void LevelCount() {
    if (gamepad2.dpad_down) {
      Level_Count = Math.min(Math.max(Level_Count - 1, 0), 9);
      Drive();
      Ticks_Per_Color();
      sleep(200);
    } else if (gamepad2.dpad_up) {
      Level_Count = Math.min(Math.max(Level_Count + 1, 0), 9);
      Drive();
      Ticks_Per_Color();
      sleep(200);
    }
  }

  /**
   * Describe this function...
   */
  private void Extending_Arm() {
    if (Is_Lift_High_Enough()) {
      if (gamepad2.dpad_right) {
        extension.setTargetPosition(Math.min(Math.max(extension.getCurrentPosition() + 70, 0), 1400));
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
        sleep(50);
        telemetry.addData("extenderboi", extension.getCurrentPosition());
      } else if (gamepad2.dpad_left) {
        extension.setTargetPosition(Math.min(Math.max(extension.getCurrentPosition() - 70, 0), 1400));
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
        sleep(50);
        telemetry.addData("extenderboi", extension.getCurrentPosition());
      } else if (false) {
        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
      } else if (false) {
        extension.setTargetPosition(900);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Gripper2() {
    if (gamepad2.y) {
      camera.setPosition(0);
    } else if (gamepad2.x) {
      camera.setPosition(280);
    }
  }

  /**
   * Describe this function...
   */
  private void Lift() {
    if (gamepad2.dpad_up) {
      lift.setDirection(DcMotorSimple.Direction.FORWARD);
      lift.setPower(1);
    } else if (false) {
    } else {
      lift.setPower(0.1);
    }
  }

  /**
   * Describe this function...
   */
  private void Ticks_Per_Color() {
    if (Level_Count == 1) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    } else if (Level_Count == 2) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    } else if (Level_Count == 3) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    } else if (Level_Count == 4) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    } else if (Level_Count == 5) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    } else if (Level_Count == 6) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    } else if (Level_Count == 7) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    } else if (Level_Count == 8) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    } else if (Level_Count == 9) {
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
  }

  /**
   * Describe this function...
   */
  private void Drop_Capstone() {
    if (gamepad2.left_bumper) {
      Speed_Multiplier = 0.35;
      lift.setTargetPosition(1450);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      sleep(1000);
      extension.setTargetPosition(1300);
      extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      extension.setPower(1);
      sleep(800);
      Gripper.setPosition(0);
      sleep(500);
      Wrist.setPosition(0.5);
      sleep(750);
      extension.setTargetPosition(0);
      extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      extension.setPower(1);
      sleep(500);
      lift.setTargetPosition(0);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      Speed_Multiplier = 0.7;
    }
  }

  /**
   * Describe this function...
   */
  private void Motor_Reset() {
    PIDFCoefficients PIDF_Lift;

    LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extension.setDirection(DcMotorSimple.Direction.REVERSE);
    extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /**
   * Describe this function...
   */
  private void Lift_Level() {
    LevelCount();
    if (false) {
      Set_Drive_Motors_To_0();
      lift.setTargetPosition(Ticks_Per_Level_Function());
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      if (Level_Count >= 5) {
        sleep(100);
      } else {
        sleep(100);
      }
      if (Is_Lift_High_Enough()) {
        if (Level_Count >= 6) {
          extension.setTargetPosition(1200);
        } else {
          extension.setTargetPosition(900);
        }
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
        sleep(500);
        lift.setTargetPosition(Level_Drop);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
      }
    } else if (gamepad2.b) {
      Speed_Multiplier = 0.35;
      Set_Drive_Motors_To_0();
      lift.setTargetPosition(Ticks_Per_Level_Function());
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      if (Level_Count >= 5) {
        sleep(100);
      } else {
        sleep(100);
      }
      if (Is_Lift_High_Enough()) {
        if (Level_Count == 6) {
          extension.setTargetPosition(1150);
        } else if (Level_Count == 1) {
          extension.setTargetPosition(1250);
        } else if (Level_Count == 3) {
          extension.setTargetPosition(1225);
        } else if (Level_Count == 5) {
          extension.setTargetPosition(1175);
        } else if (Level_Count == 4) {
          extension.setTargetPosition(1200);
        } else if (Level_Count == 7) {
          extension.setTargetPosition(1100);
        } else if (Level_Count >= 8) {
          extension.setTargetPosition(1050);
        } else {
          extension.setTargetPosition(1250);
        }
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1);
        if (Level_Count > 2) {
          sleep(500);
        } else {
          sleep(800);
        }
        lift.setTargetPosition(Level_Drop);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
      }
    } else if (gamepad2.x) {
      Set_Drive_Motors_To_0();
      Gripper.setPosition(0);
      sleep(300);
      if (Level_Count <= 2 || Cappstone_Down == 1) {
        lift.setTargetPosition(Ticks_Per_Level_Function() + 200);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        sleep(500);
      }
      extension.setTargetPosition(1);
      extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      if (Level_Count >= 6) {
        extension.setPower(0.5);
      } else {
        extension.setPower(0.75);
      }
      while (extension.getCurrentPosition() > 75) {
      }
      Gripper.setPosition(1);
      Wrist.setPosition(0.76);
      lift.setTargetPosition(0.5);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      Level_Count += 1;
      Speed_Multiplier = 0.7;
    } else if (false) {
    } else if (gamepad2.left_stick_y != 0) {
      Drive();
      while (gamepad2.left_stick_y != 0) {
        Lift_Position = Math.min(Math.max(lift.getCurrentPosition() + gamepad2.left_stick_y * -20, 0), 3500);
        lift.setTargetPosition(Lift_Position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        Drive();
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Lift Target Var", Lift_Position);
        telemetry.addData("Lift Target ", lift.getTargetPosition());
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private int Ticks_Per_Level_Function() {
    if (Level_Count == 1) {
      Level_Ticks = 1300;
      Level_Drop = 250;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    } else if (Level_Count == 2) {
      Level_Ticks = 1300;
      Level_Drop = 800;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    } else if (Level_Count == 3) {
      Level_Ticks = 1450;
      Level_Drop = 1200;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    } else if (Level_Count == 4) {
      Level_Ticks = 1850;
      Level_Drop = 1575;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    } else if (Level_Count == 5) {
      Level_Ticks = 2150;
      Level_Drop = 1950;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    } else if (Level_Count == 6) {
      Level_Ticks = 2650;
      Level_Drop = 2350;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    } else if (Level_Count == 7) {
      Level_Ticks = 2950;
      Level_Drop = 2750;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    } else if (Level_Count == 8) {
      Level_Ticks = 3300;
      Level_Drop = 3150;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    } else if (Level_Count == 9) {
      Level_Ticks = 3800;
      Level_Drop = 3450;
      LitLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    return Level_Ticks;
  }

  /**
   * Describe this function...
   */
  private void Lift_Early() {
    lift.setTargetPosition(Ticks_Per_Level_Function());
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(1);
  }

  /**
   * Describe this function...
   */
  private void Foundation() {
    if (gamepad1.dpad_up) {
      FoundationRight.setPosition(0.88);
      FoundationLeft.setPosition(0.07);
    } else if (gamepad1.dpad_down) {
      FoundationRight.setPosition(0.4);
      FoundationLeft.setPosition(0.52);
    }
  }

  /**
   * Describe this function...
   */
  private void Set_Drive_Motors_To_0() {
    LeftFront.setPower(0);
    RightFront.setPower(0);
    LeftRear.setPower(0);
    RightRear.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Capstone2() {
    if (gamepad2.y) {
      Capstone.setPosition(0);
      Capstone.setPosition(0.2);
      FoundationRight.setPosition(0.88);
      FoundationLeft.setPosition(0.07);
      Drive();
    }
  }
}
