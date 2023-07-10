#include <ros.h>
#include <std_msgs/Float64.h>

#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define BUTTON_PIN 2

ros::NodeHandle nh;
std_msgs::Float64 z_val, r_val, t_val, g_val;
ros::Publisher pub_zvalue("/zvalue", &z_val);
ros::Publisher pub_rvalue("/rvalue", &r_val);
ros::Publisher pub_tvalue("/tvalue", &t_val);
ros::Publisher pub_gvalue("/gvalue", &g_val);

const float max_l = 210;
const int deadzone = 30;

float r_inc_rate = 0, z_inc_rate = 0, t_inc_rate = 0, g_inc_rate = 0;
float r = 0, z = 210, t = 0, g = 0;
byte mode = 0;

void Change_Mode() {
  mode ^= 1;
}

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_zvalue);
  nh.advertise(pub_rvalue);
  nh.advertise(pub_tvalue);
  nh.advertise(pub_gvalue);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), Change_Mode, FALLING);
}

void loop() {
  int x = analogRead(JOYSTICK_X) - 512;
  int y = analogRead(JOYSTICK_Y) - 512;

  if (mode == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (abs(x) > deadzone)
      r_inc_rate = -0.1 * x / 512.0;
    else
      r_inc_rate = 0;

    if (abs(y) > deadzone)
      t_inc_rate = -0.3 * y / 512.0;
    else
      t_inc_rate = 0;

    r += r_inc_rate;
    t += t_inc_rate;

    if ((r * r + z * z) >= (max_l * max_l))
      r -= r_inc_rate;

    if (t > 180 || t < -180)
      t -= t_inc_rate;
  } else if (mode == 1) {
    digitalWrite(LED_BUILTIN, LOW);
    if (abs(x) > deadzone)
      z_inc_rate = -0.1 * x / 512.0;
    else
      z_inc_rate = 0;

    if (abs(y) > deadzone)
      g_inc_rate = -0.3 * y / 512.0;
    else
      g_inc_rate = 0;

    g += g_inc_rate;
    z += z_inc_rate;

    if ((r * r + z * z) >= (max_l * max_l))
      z -= z_inc_rate;

    if (g > 180 || g < -180)
      g -= g_inc_rate;
  }
  
  t_val.data = t;
  r_val.data = r;
  g_val.data = g;
  z_val.data = z;

  pub_tvalue.publish(&t_val);
  pub_rvalue.publish(&r_val);
  pub_gvalue.publish(&g_val);
  pub_zvalue.publish(&z_val);
  nh.spinOnce();
}
