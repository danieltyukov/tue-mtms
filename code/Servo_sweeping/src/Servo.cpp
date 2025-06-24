#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Servo.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <movement_msgs/srv/set_angle.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>

#include <rosidl_runtime_c/string_functions.h>

rcl_service_t service;
rcl_publisher_t publisher;

rcl_subscription_t right_motor;
rcl_subscription_t left_motor;

std_msgs__msg__Float32 right_msg;
std_msgs__msg__Float32 left_msg;

rclc_executor_t executor;
rcl_node_t node;

rcl_allocator_t allocator;
rclc_support_t support;

// std_srvs__srv__SetBool_Request req;
// std_srvs__srv__SetBool_Response res;

movement_msgs__srv__SetAngle_Request req;
movement_msgs__srv__SetAngle_Response res;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int init_servo_angle_left = 0;
int init_servo_angle_right = 180;



int SERVO1_PIN = 53;
int SERVO2_PIN = 51;
int SERVO3_PIN = 49;
int SERVO4_PIN = 47;

// Right Motors
int MOTOR1_PWM_PIN = 2;
int MOTOR1_DIR_PIN = 6;
int MOTOR1_FG_PIN = 11;

int MOTOR2_PWM_PIN = 3;
int MOTOR2_DIR_PIN = 7;
int MOTOR2_FG_PIN = 10;

// Left Motors
int MOTOR3_PWM_PIN = 4;
int MOTOR3_DIR_PIN = 8;
int MOTOR3_FG_PIN = 12;
    
int MOTOR4_PWM_PIN = 5;
int MOTOR4_DIR_PIN = 9;
int MOTOR4_FG_PIN = 13;

void right_callback(const void *msgin) {
    // digitalWrite(12, HIGH);
    // analogWrite(A5, 0);
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float speed = msg->data;

    if (speed >= 0) {
        digitalWrite(MOTOR1_DIR_PIN, LOW);
        digitalWrite(MOTOR2_DIR_PIN, LOW);
    } else {
        digitalWrite(MOTOR1_DIR_PIN, HIGH);
        digitalWrite(MOTOR2_DIR_PIN, HIGH);
    }

    analogWrite(MOTOR1_PWM_PIN, 255 - abs(speed));
    analogWrite(MOTOR2_PWM_PIN, 255 - abs(speed));
}

void left_callback(const void *msgin) {

    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float speed = msg->data;

    if (speed >= 0) {
        digitalWrite(MOTOR3_DIR_PIN, LOW);
        digitalWrite(MOTOR4_DIR_PIN, LOW);
    } else {
        digitalWrite(MOTOR3_DIR_PIN, HIGH);
        digitalWrite(MOTOR4_DIR_PIN, HIGH);
    }

    analogWrite(MOTOR3_PWM_PIN, 255 - abs(speed));
    analogWrite(MOTOR4_PWM_PIN, 255 - abs(speed));
}

void service_callback(const void *request_msg, void *response_msg) {
    const movement_msgs__srv__SetAngle_Request *req_in =
        (const movement_msgs__srv__SetAngle_Request *)request_msg;
    movement_msgs__srv__SetAngle_Response *res_out =
        (movement_msgs__srv__SetAngle_Response *)response_msg;

    int angle = req_in->angle;

    if (angle >= 0 && angle <= 180) {
        servo1.write(angle);
        servo2.write(angle);
        servo3.write(180 - angle);
        servo4.write(180 - angle);
        delay(1000);
        res_out->success = true;
        rosidl_runtime_c__String__assign(&res_out->message, "Moved to angle");
    } else {
        res_out->success = false;
        rosidl_runtime_c__String__assign(&res_out->message,
                                         "Angle out of range");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Attach servos
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);

    // Setup dc motors
    // Set pinModes
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    pinMode(MOTOR3_PWM_PIN, OUTPUT);
    pinMode(MOTOR4_PWM_PIN, OUTPUT);

    pinMode(MOTOR1_FG_PIN, INPUT_PULLUP);
    pinMode(MOTOR2_FG_PIN, INPUT_PULLUP);
    pinMode(MOTOR3_FG_PIN, INPUT_PULLUP);
    pinMode(MOTOR4_FG_PIN, INPUT_PULLUP);

    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_DIR_PIN, OUTPUT);
    pinMode(MOTOR4_DIR_PIN, OUTPUT);

    // Initialize motors and servos to correct positions
    analogWriteResolution(8);
    analogWrite(MOTOR1_PWM_PIN, 255);
    analogWrite(MOTOR2_PWM_PIN, 255);
    analogWrite(MOTOR3_PWM_PIN, 255);
    analogWrite(MOTOR4_PWM_PIN, 255);

    digitalWrite(MOTOR1_DIR_PIN, LOW);
    digitalWrite(MOTOR2_DIR_PIN, LOW);
    digitalWrite(MOTOR3_DIR_PIN, LOW);
    digitalWrite(MOTOR4_DIR_PIN, LOW);

    servo1.write(init_servo_angle_left);
    servo2.write(init_servo_angle_left);
    servo3.write(init_servo_angle_right);
    servo4.write(init_servo_angle_right);

    std_msgs__msg__Float32__init(&right_msg);
    std_msgs__msg__Float32__init(&left_msg);

    // Setup Micro-ROS
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_node", "", &support);

    // Initialize subscriptions and services
    rclc_subscription_init_default(
        &right_motor, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive_right");
    rclc_subscription_init_default(
        &left_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "drive_left");

    rclc_service_init_default(
        &service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(movement_msgs, srv, SetAngle),
        "servo_service");

    // Initialize executor and allocate subs and srvs to executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &right_motor, &right_msg,
                                   &right_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &left_motor, &left_msg,
                                   &left_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &service, &req, &res,
                              service_callback);
}

void loop() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); }
