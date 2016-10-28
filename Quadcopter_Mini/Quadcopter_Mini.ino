#include "GLOBAL.h"
#define DEBUG_MODE false


int operation_mode = 1;
bool gps_global_flag = false;


cSerial Uart;
cImu Imu;
cController Controller;
cESP Esp;
float automatic_lift_I = 0;

tCommand command = {0, 0, 0, 0};



void setup()
{
  pinMode(A0, INPUT);
  digitalWrite(A0, 1);
  operation_mode = digitalRead(A0);


  if (operation_mode == 1)
  {
    ////// Controller Init
    Controller.begin();
    Controller.armMotors();
    ////// Controller Init

    ////// IMU Init
    Imu.begin();
    delay(50);
    if (Imu.isValid() == false) {
      Serial.begin(115200);
      while (Imu.isValid() == false)
      {
        delay(1000);
        blink(3);
        Serial.println("No Imu");
      }
    }
    ////// IMU Init


    ////// ESP Init
    Esp.begin();
    delay(50);
    if (Esp.isValid() == false) {
      Serial.begin(115200);
      while (Esp.isValid() == false)
      {
        delay(1000);
        blink(3);
        Serial.println("No Esp");
      }
    }


    Esp.setupAP();
    blink(2);
    while ( !Esp.isConnected() )
    {
      delay(100);
    }
    blink(3);
    ////// ESP Init


    Imu.gyro_calibration();
    safe_mode();


  }


  //////////////////////////////////////////////// DEBUG MODE
  if (operation_mode == 0)
  {
    Serial.begin(115200);
    Serial.println("Setup Mode...");
    blink(3);
    while (operation_mode == 0)
    {



    }

  }
  //////////////////////////////////////////////// DEBUG MODE


}




float time_wo_command = 0, time_wo_command_th = 1;

void loop()
{
  // Get Sensor Data and Attitude
  Imu.update();

  //Process Command
  process_command();

  if (gps_global_flag == true) // Override the Pilot Command
  {
    override_pilot_command();
  }

  // Calculate the Pseudo Control
  Controller.calculatePseudoControl(&Imu, &command);

  // Control Allocation
  Controller.controlAllocation();

  if (DEBUG_MODE)
  {
    // Test Speed of Controller
    static int i = 0;
    static float mean_dt = 0;
    mean_dt += Imu.dt;
    i++;
    if (i >= 1000)
    {
      mean_dt /= i;
      Serial.println(mean_dt, 5);    
      mean_dt = 0;
      i = 0;
    }
  }


}










void process_command()
{
  /////////////////////////////////////////////////////Process Command
  // Check if a new commando has been received....
  if ( Esp.getCommand(command))
  {
    time_wo_command = 0;
    PORTB &= ~(1 << PB5);
    
    if (command.r <= -50)
    {
      gps_global_flag = false;
      automatic_lift_I = 0;
    }

    /////////////////////////////////////////////////// OFF Command received
    if (command.T <= -100 && command.r <= -100)
    {



      Controller.pseudo_control.M[0] = 0;
      Controller.pseudo_control.M[1] = 0;
      Controller.pseudo_control.M[2] = 0;
      Controller.pseudo_control.T = 0;
      // Control Allocation
      Controller.controlAllocation();

      safe_mode();

    }
    /////////////////////////////////////////////////// OFF Command received



  }
  // ...check if too much time has passed
  else
  {
    time_wo_command += Imu.dt;
    if (time_wo_command > time_wo_command_th)
    {
      PORTB |= (1 << PB5);
      command.q_BI_x = 0 ;
      command.q_BI_x = 0 ;
      command.T = -110;
      gps_global_flag = false;
    }

  }
  /////////////////////////////////////////////////////Process Command

}


void safe_mode()
{
  // Start Routine
  bool start_cmd_received = false;
  long blink_t = millis();
  DDRB |= (1 << PB5);
  int blink_time_ms = 20;
  while (!start_cmd_received)
  {
    Esp.getCommand(command);
    if (command.T <= -100 && command.r >= 100)     // left TRIGGER + right DOWN
      start_cmd_received = true;

    if (command.q_BI_y <= -100 && command.r >= 100) // left DOWN + right DOWN
      Imu.calibrate();

    if (command.q_BI_y >= 100 && command.r <= -100) // left UP + right UP
      Imu.calibrate(false);

    if (command.T >= 100)     // Right TRIGGER
      set_gps_flag();

    if (millis() - blink_t <= blink_time_ms)
    {
      PORTB |= (1 << PB5);
    }

    if (millis() - blink_t > blink_time_ms)
      PORTB &= ~(1 << PB5);

    if (millis() - blink_t >= 700)
      blink_t = millis();

    Imu.update();
    if (Imu.gps_package.isGPSvalid)
    {
      blink_time_ms = 150;
      Imu.gps_package_from_ground = Imu.gps_package;
    }
    else
    {
      blink_time_ms = 20;
    }
  }
  PORTB &= ~(1 << PB5);
  blink(1);
  // Start Routine
}




void set_gps_flag()
{
  bool flag_to_set;
  if (Imu.gps_package.isGPSvalid)
  {
    if (gps_global_flag == false)
    {
      flag_to_set = true;
      fastblink(5);
      command.T = 0;
      Uart.flushBuffer();
      delay(300);
    }

    if (gps_global_flag == true)
    {
      flag_to_set = false;
      fastblink(3);
      command.T = 0;
      Uart.flushBuffer();
      delay(300);
    }
  }
  else
  {
    flag_to_set = false;
    fastblink(3);
    command.T = 0;
    Uart.flushBuffer();
    delay(300);
  }
  gps_global_flag = flag_to_set;

}



void override_pilot_command()
{
  cVector<3> t, x, xdot, x_cmd;
  float Kd = 3, Kp = 5 , KI=5;
  tCommand command_to_send;

  x(1) = Imu.gps_package.delta_NED[0] - Imu.gps_package_from_ground.delta_NED[0];
  x(2) = Imu.gps_package.delta_NED[1] - Imu.gps_package_from_ground.delta_NED[1];
  x(3) = Imu.gps_package.delta_NED[2] - Imu.gps_package_from_ground.delta_NED[2];

  xdot(1) = Imu.gps_package.delta_UVW[0] - Imu.gps_package_from_ground.delta_UVW[0];
  xdot(2) = Imu.gps_package.delta_UVW[1] - Imu.gps_package_from_ground.delta_UVW[1];
  xdot(3) = Imu.gps_package.delta_UVW[2] - Imu.gps_package_from_ground.delta_UVW[2];

  // Hover 4m above ground
  x_cmd(1) = Imu.gps_package_from_ground.delta_NED[0];
  x_cmd(2) = Imu.gps_package_from_ground.delta_NED[1];
  x_cmd(3) = Imu.gps_package_from_ground.delta_NED[2] + 4;

  t = -Kd*xdot - Kp*(x-x_cmd);
  automatic_lift_I += Imu.dt*KI*( x_cmd(3) -  x(3) );
  
  t(3) += automatic_lift_I;
  
  if ( t(3) < 0)
  t(3) = 0;
  float t_norm = t.norm();
  if (t_norm < 0.01)
  t_norm = 0.01;
  
  //command_to_send.T = t_norm - 110;
  command_to_send.T = command.T;
  
  command_to_send.q_BI_x = (t(2))*(-3);
  command_to_send.q_BI_y = (t(1))*(3);
  
  
  // Limits for the commands
  if (command_to_send.T >= UPPER_LIMIT_THRUST_CMD || command_to_send.T < LOWER_LIMIT_THRUST_CMD)
    command_to_send.T = command_to_send.T >= UPPER_LIMIT_THRUST_CMD ? UPPER_LIMIT_THRUST_CMD : LOWER_LIMIT_THRUST_CMD;

  if (command_to_send.q_BI_x >= UPPER_LIMIT_Q_CMD || command_to_send.q_BI_x < LOWER_LIMIT_Q_CMD)
    command_to_send.q_BI_x = command_to_send.q_BI_x >= UPPER_LIMIT_Q_CMD ? UPPER_LIMIT_Q_CMD : LOWER_LIMIT_Q_CMD;

  if (command_to_send.q_BI_y >= UPPER_LIMIT_Q_CMD || command_to_send.q_BI_y < LOWER_LIMIT_Q_CMD)
    command_to_send.q_BI_y = command_to_send.q_BI_y >= UPPER_LIMIT_Q_CMD ? UPPER_LIMIT_Q_CMD : LOWER_LIMIT_Q_CMD;

  command = command_to_send;

}


