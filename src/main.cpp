#include "rclcpp/rclcpp.hpp"
#include "kk_driver_msg/msg/encoder.hpp"
//#include "kk_driver_msg/msg/mouse_ctrl.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "kk_driver_msg/msg/bldc_cmd.hpp"
#include "kk_driver_msg/msg/c610_cmd.hpp"
//#include "kk_driver_msg/msg/camera_cmd.hpp"
#include "kk_driver_msg/msg/c610_status.hpp"
#include "kk_driver_msg/msg/core.hpp"
#include "sensor_msgs/msg/joy.hpp"

class core_node : public rclcpp::Node{
public:
    rclcpp::Subscription<kk_driver_msg::msg::Core>::SharedPtr sub_joy;
    //rclcpp::Subscription<kk_driver_msg::msg::MouseCtrl>::SharedPtr sub_mou;
    rclcpp::Subscription<kk_driver_msg::msg::Encoder>::SharedPtr sub_enc;
    rclcpp::Subscription<kk_driver_msg::msg::C610Status>::SharedPtr c610_enc;
    rclcpp::Publisher<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::BldcCmd>::SharedPtr bldc_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::C610Cmd>::SharedPtr c610_cmd_sub;
    //rclcpp::Publisher<kk_driver_msg::msg::CameraCmd>::SharedPtr camera_cmd_sub;

    kk_driver_msg::msg::MotorCmd motor1_cmd_msg;
    kk_driver_msg::msg::MotorCmd motor2_cmd_msg;
    kk_driver_msg::msg::MotorCmd motor3_cmd_msg;
    kk_driver_msg::msg::BldcCmd bldc_cmd_msg;
    kk_driver_msg::msg::C610Cmd c610_cmd_msg;
    //kk_driver_msg::msg::CameraCmd camera_cmd_msg;

    core_node() : Node("core_node_node"){

        // Joyコンのサブスクライバー
        sub_joy = this->create_subscription<kk_driver_msg::msg::Core>(
            "/core", rclcpp::QoS(10),
            std::bind(&core_node::core_callback, this, std::placeholders::_1)
        );
        // キーボードのサブスクライバー (CoRE2用)
        /*sub_key = this->create_subscription<kk_driver_msg::msg::MouseCtrl>(
            "/mouses" , rclcpp::QoS(10),
            std::bind(&core_node::mouse_callback , this, std::placeholders::_1)
        );*/
        //エンコーダのサブスクライバー
        sub_enc = this->create_subscription<kk_driver_msg::msg::Encoder>(
            "/mtr/encoder", rclcpp::QoS(10),
            std::bind(&core_node::enc_callback , this, std::placeholders::_1)
        );
        c610_enc = this->create_subscription<kk_driver_msg::msg::C610Status>(
            "/c610/status", rclcpp::QoS(10),
            std::bind(&core_node::c610_callback , this, std::placeholders::_1)
        );
        motor_cmd_sub = this->create_publisher<kk_driver_msg::msg::MotorCmd>("mtr/cmd", rclcpp::QoS(10));
        bldc_cmd_sub = this->create_publisher<kk_driver_msg::msg::BldcCmd>("bldc/cmd", rclcpp::QoS(10));
        c610_cmd_sub = this->create_publisher<kk_driver_msg::msg::C610Cmd>("c610/cmd", rclcpp::QoS(10));
        //camera_cmd_sub = this->create_publisher<kk_driver_msg::msg::CameraCmd>("camera/cmd", rclcpp::QoS(10));
    };
    
    bool test_count = false;
    bool mode = false;
    int launch = 0;
    int begin = 0;
    int turn_ok = 0;
    float x_duty;
    float y_duty;
    float duty[4]={0.0f};
    float tt_duty = 0x0F4240;
    float front_e;
    float rear_e;
    float right_e;
    float left_e;
    float turn_s;
    float turn_e;
    float turn_t;
    float turn_c;
    float turn_d;
    float turn_m;
    float turn_u;
    float integral;
    float derivative;
    float prev_error;
    float c610_e;
    float c610_t;
    float c610_f;
    int camera = 0;
    int X, Y = 0;  // 計算に使う座標

    float br_left;
    float br_right;

    const float dt = 0.05;
    const float Kp = 0.1;
    const float Ki = 0.01;
    const float Kd = 0.05;

    int32_t encValidate(int32_t val){
        if (val > 0x8000)
            return val - 0xFFFF;
        else 
            return val; 
    }

    void c610_callback(const kk_driver_msg::msg::C610Status::SharedPtr msg){
        c610_e = msg->position[0];
        //printf("c610_e=%f\n", c610_e);
    }

    void enc_callback(const kk_driver_msg::msg::Encoder::SharedPtr msg){
        if(msg->child_id == 0){
            right_e = msg->pos[0];
            rear_e = msg->pos[1];   // エンコーダの位置データ 生パルス数です
        }

        if(msg->child_id == 1){
            front_e = msg->pos[0];
            left_e = msg->pos[1];
        }

        if(msg->child_id == 2){
            turn_e = msg->pos[0];
            c610_e = msg->pos[1];
        };   // 基板CAN子ID
        
    }

    void Yonrin(float x, float y)       //四輪オムニのpwm値出し
    {
        duty[0] = -y;
        duty[1] = -x;
        duty[2] = -x;
        duty[3] = y;
        //printf("前=%f , 右=%f ,後=%f ,左=%f\n", duty[2], duty[0], duty[1], duty[3]);

        // auto motor_cmd_msg = kk_driver_msg::msg::MotorCmd();

        //送信するポート数に合わせて配列長を設定
        motor1_cmd_msg.child_id = 0;
        motor1_cmd_msg.port.resize(2);
        motor1_cmd_msg.ctrl.resize(2);
        motor1_cmd_msg.target.resize(2);

        motor2_cmd_msg.child_id = 1;
        motor2_cmd_msg.port.resize(2);
        motor2_cmd_msg.ctrl.resize(2);
        motor2_cmd_msg.target.resize(2);
        
        motor1_cmd_msg.port[0] = 0;
        motor1_cmd_msg.ctrl[0] = 1;
        motor1_cmd_msg.target[0] = static_cast<int32_t>(duty[0] * 0x7FFF);

        motor1_cmd_msg.port[1] = 1;
        motor1_cmd_msg.ctrl[1] = 1;
        motor1_cmd_msg.target[1] = static_cast<int32_t>(duty[1] * 0x7FFF);

        motor2_cmd_msg.port[0] = 0;
        motor2_cmd_msg.ctrl[0] = 1;
        motor2_cmd_msg.target[0] = static_cast<int32_t>(duty[2] * 0x7FFF);

        motor2_cmd_msg.port[1] = 1;
        motor2_cmd_msg.ctrl[1] = 1;
        motor2_cmd_msg.target[1] = static_cast<int32_t>(duty[3] * 0x7FFF);

    }

    void Senkai(int s)       //四輪オムニのpwm値出し
    {
        if(s == 1){
            duty[0] = 130;
            duty[1] = -130;
            duty[2] = 130;
            duty[3] = 130;
        }
        if(s == -1){
            duty[0] = -130;
            duty[1] = 130;
            duty[2] = -130;
            duty[3] = -130;
        }
        
        //printf("前=%f , 右=%f ,後=%f ,左=%f\n", duty[2], duty[0], duty[1], duty[3]);

        // auto motor_cmd_msg = kk_driver_msg::msg::MotorCmd();

        //送信するポート数に合わせて配列長を設定
        motor1_cmd_msg.child_id = 0;
        motor1_cmd_msg.port.resize(2);
        motor1_cmd_msg.ctrl.resize(2);
        motor1_cmd_msg.target.resize(2);

        motor2_cmd_msg.child_id = 1;
        motor2_cmd_msg.port.resize(2);
        motor2_cmd_msg.ctrl.resize(2);
        motor2_cmd_msg.target.resize(2);
        
        motor1_cmd_msg.port[0] = 0;
        motor1_cmd_msg.ctrl[0] = 1;
        motor1_cmd_msg.target[0] = static_cast<int32_t>(duty[0] * 0x4FFF);

        motor1_cmd_msg.port[1] = 1;
        motor1_cmd_msg.ctrl[1] = 1;
        motor1_cmd_msg.target[1] = static_cast<int32_t>(duty[1] * 0x4FFF);

        motor2_cmd_msg.port[0] = 0;
        motor2_cmd_msg.ctrl[0] = 1;
        motor2_cmd_msg.target[0] = static_cast<int32_t>(duty[2] * 0x4FFF);

        motor2_cmd_msg.port[1] = 1;
        motor2_cmd_msg.ctrl[1] = 1;
        motor2_cmd_msg.target[1] = static_cast<int32_t>(duty[3] * 0x4FFF);

    }


    // Joyコンの処理
    void core_callback(const kk_driver_msg::msg::Core::SharedPtr msg){
        turn_c = encValidate(turn_e);
        printf("turn_c=%f\n", turn_c);
        
        if(msg->limit == 0){
            tt_duty = 0;
            turn_s = turn_c;
            printf("turn_s=%f\n", turn_s);
            turn_s = turn_s + 1900;
            turn_ok = 1;
        }

        if(turn_ok == 1){
            tt_duty = -0x0F4240;
            if (turn_c >= turn_s){
                tt_duty = 0;
                turn_s = turn_c;
                printf("turn_s=%f\n", turn_s);
                turn_ok = 2;
            }
        }

        if(turn_ok == 2){
            //turn_d = msg->cmd[2] - 0x7F;
            turn_m = turn_s + (msg->cmd[2] - 0x7F);
            turn_d = turn_m - turn_c;
            integral += turn_d * dt;
            derivative = (turn_d - prev_error) / dt;
            prev_error = turn_d;
            turn_u = Kp * turn_d + Ki * integral + Kd * derivative;
            printf("turn_u=%f\n", turn_u);
            printf("turn_m=%f, turn_c=%f,turn_d=%f\n", turn_m, turn_c, turn_d);
            if(turn_c >= turn_s  && turn_c <= (turn_s + 1600)){
                tt_duty = -(turn_u * 0x1000);
            }else if(turn_c < turn_s && turn_c >= (turn_s - 1600)){
                tt_duty = -(turn_u * 0x1000);
            }else{
                tt_duty = 0x0F4240;
                turn_ok = 0;
            }
        }

        motor3_cmd_msg.child_id = 2;
        motor3_cmd_msg.port.resize(1);
        motor3_cmd_msg.ctrl.resize(1);
        motor3_cmd_msg.target.resize(1);

        motor3_cmd_msg.port[0] = 0;
        motor3_cmd_msg.ctrl[0] = 1;
        motor3_cmd_msg.target[0] = static_cast<int32_t>(tt_duty);

        if(msg->cmd[3] < 0x7A || msg->cmd[3] > 0x84 || msg->cmd[4] < 0x7A || msg->cmd[4] > 0x84){ //4輪オムニ
            x_duty = (msg->cmd[3] - 0x7F);
            y_duty = (msg->cmd[4] - 0x7F);

            Yonrin(x_duty, y_duty);
        }else{
            x_duty = 0;
            y_duty = 0;

            Yonrin(x_duty, y_duty);
        }
        if(msg->cmd[7] == 0x02){
            Senkai(1);
        }
        if(msg->cmd[7] == 0x08){
            Senkai(-1);
        }

        //printf("x_duty=%f, y_duty=%f\n", x_duty, y_duty);

        /*if(begin == 0 && msg->cmd[1] == 0x00){
            br_left = 0x2FFF;
            br_right = 0x2FFF;

            bldc_cmd_msg.child_id = 0;
            bldc_cmd_msg.port.resize(2);
            bldc_cmd_msg.spd.resize(2);

            bldc_cmd_msg.port[0] = 0;
            bldc_cmd_msg.spd[0] = br_left;

            bldc_cmd_msg.port[1] = 1;
            bldc_cmd_msg.spd[1] = br_right;
            bldc_cmd_sub->publish(bldc_cmd_msg);

            sleep(1.0);

            begin = 1;
        }*/

        if (msg->cmd[1] == 0x04 || msg->cmd[1] == 0x05 ){ //射出モード切替
            br_left = 0xFFFF;
            br_right = 0x0000;
        } else if(msg->cmd[7] == 0x04){
            br_left = 0x000000;
            br_right = 0x000000;
            begin = 0;
        } else {
            br_left = 0x2FFF;
            br_right = 0x0000;
        }

        bldc_cmd_msg.child_id = 0;
        bldc_cmd_msg.port.resize(2);
        bldc_cmd_msg.spd.resize(2);

        bldc_cmd_msg.port[0] = 0;
        bldc_cmd_msg.spd[0] = br_left;

        bldc_cmd_msg.port[1] = 1;
        bldc_cmd_msg.spd[1] = br_right;

        //printf("br_left=%f, br_right=%f\n", br_left, br_right);

        if (msg->cmd[1] == 0x06 || msg->cmd[1] == 0x07 ){
            if(launch == 0){

                printf("OK\n");
                c610_cmd_msg.child_id = 0;
                c610_cmd_msg.port.resize(1);
                c610_cmd_msg.torque.resize(1);

                // 複数ポート分の設定
            
                c610_cmd_msg.port[0] = 0;// ポート0のサーボ
                c610_cmd_msg.torque[0] = 0x05DC; // ポート0の指令値
                c610_cmd_sub->publish(c610_cmd_msg);

                sleep(1.0);

                c610_cmd_msg.port[0] = 0;// ポート0のサーボ
                c610_cmd_msg.torque[0] = 0x0000; // ポート0の指令値
                c610_cmd_sub->publish(c610_cmd_msg);

                launch = 1;
            }
        }else{
            launch = 0;
            c610_cmd_msg.child_id = 0;
            c610_cmd_msg.port.resize(1);
            c610_cmd_msg.torque.resize(1);

            // 複数ポート分の設定
            c610_cmd_msg.port[0] = 0;// ポート0のサーボ
            c610_cmd_msg.torque[0] = 0x0000; // ポート0の指令値
        }

        if(msg->cmd[7] == 0x80){
            camera = camera - 1;
            if(camera < 0){
                camera = 3;
            }
        }else if(msg->cmd[7] == 0x20){
            camera = camera + 1;
            if(camera > 3){
                camera = 0;
            }
        }
        //camera_cmd_msg.child_id = 0;
        //camera_cmd_msg.port[0] = camera;
       
        motor_cmd_sub->publish(motor1_cmd_msg);
        motor_cmd_sub->publish(motor2_cmd_msg);
        motor_cmd_sub->publish(motor3_cmd_msg);
        bldc_cmd_sub->publish(bldc_cmd_msg);
        c610_cmd_sub->publish(c610_cmd_msg);
        //camera_cmd_sub->publish(camera_cmd_msg);

    }

    /* マウス入力の処理
    void mouse_callback(const kk_driver_msg::msg::MouseCtrl::SharedPtr msg){

        printf("keys\n");

        クライアントからの変位
        int Xpoti = msg->x;
        //int Ypoti = msg->y;

        int X_limit = 2000;//最大値、最小値を設定
        if(Xpoti > X_limit || Xpoti < X_limit*-1){
            if( Xpoti > X_limit){
                Xpoti = X_limit;
            }else if(Xpoti < X_limit*-1){
                Xpoti = X_limit*-1;
            }
        }
        printf("X : %d\n" , Xpoti);

        auto pwm_cmd_msg = kk_driver_msg::msg::PwmCmd();
        pwm_cmd_msg.child_id = 0;
        pwm_cmd_msg.port = {0};
        pwm_cmd_msg.pos.resize(1); 
        pwm_cmd_msg.pos[0]= pulse_calculate( X , max_min);
        pwm_cmd_msg.spd= {0xdf};
        pub_pwm_cmd.publish(pwm_cmd_msg);

        motor3_cmd_msg.child_id = 2;
        motor3_cmd_msg.port.resize(2);
        motor3_cmd_msg.ctrl.resize(2);
        motor3_cmd_msg.target.resize(2);

        motor3_cmd_msg.port[0] = 0;
        motor3_cmd_msg.ctrl[0] = 1;
        motor3_cmd_msg.target[0] = static_cast<int32_t>(tt_rot * 0x3FFFFF);

        if (msg->keys[4]){ //射出モード切替
            br_left = 0x3FFFFF;
            br_right = 0x3FFFFF;
        } else {
            br_left = 0x0FFFFF;
            br_right = 0x0FFFFF;
        }

        bldc_cmd_msg.child_id = 0;
        bldc_cmd_msg.port.resize(2);
        bldc_cmd_msg.spd.resize(2);

        bldc_cmd_msg.port[0] = 0;
        bldc_cmd_msg.spd[0] = br_left;

        bldc_cmd_msg.port[1] = 1;
        bldc_cmd_msg.spd[1] = br_right;

        printf("br_left=%f, br_right=%f\n", br_left, br_right);

        if(msg->keys[6] && msg->keys[4]){ //発射
            printf("push\n");
            while(launch < 360){
                launch = launch + 1;

                c610_cmd_msg.child_id = 0;
                c610_cmd_msg.port.resize(1);
                c610_cmd_msg.torque.resize(1);

                // 複数ポート分の設定
                c610_cmd_msg.port[0] = 0;// ポート0のサーボ
                c610_cmd_msg.torque[0] = 0x000F; // ポート0の指令値

                c610_cmd_sub->publish(c610_cmd_msg);
                
                printf("launch=%d\n", launch);

            }
            c610_cmd_msg.child_id = 0;
            c610_cmd_msg.port.resize(1);
            c610_cmd_msg.torque.resize(1);

            // 複数ポート分の設定
            c610_cmd_msg.port[0] = 0;// ポート0のサーボ
            c610_cmd_msg.torque[0] = 0x00; // ポート0の指令値

            c610_cmd_sub->publish(c610_cmd_msg);
            launch = 0;
        }

        motor_cmd_sub->publish(motor1_cmd_msg);
        motor_cmd_sub->publish(motor2_cmd_msg);
        motor_cmd_sub->publish(motor3_cmd_msg);
        bldc_cmd_sub->publish(bldc_cmd_msg);
        c610_cmd_sub->publish(c610_cmd_msg);

    }
    
    //マウスの位置と最大値、最小値
    double pulse_calculate( int AmousePoti , int Amax_min){
        int result_pulse = 1500 + 500 * AmousePoti/Amax_min;
        return result_pulse;
    }*/

};


int main(int argc, char *argv[]) {
    // ROS 2クライアントライブラリの初期化
    rclcpp::init(argc, argv);
    // ノードの作成と実行
    auto node = std::make_shared<core_node>();
    // スピン処理
    rclcpp::spin(node);
    // ROS 2クライアントライブラリの終了処理
    rclcpp::shutdown();
    return 0;
}