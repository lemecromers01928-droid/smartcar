#include "main.h" // 所有引脚信息更改在main.h里改宏
#include <math.h>

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
    pit_ms_init(PIT_CH0, 10);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];

static float last_mid_err = 0.0f;
static uint32_t telemetry_frame = 0;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    Init();
    interrupt_global_enable(0);

    while (1)
    {
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            int threshold_adjusted = (int)get_threshold_otsu(hist_gram) + RACE_THRESHOLD_OFFSET;
            unsigned char threshold;
            if (threshold_adjusted < 1) threshold_adjusted = 1;
            if (threshold_adjusted > 254) threshold_adjusted = 254;
            threshold = (unsigned char)threshold_adjusted;
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);

            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][mid_line[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 100);

            // 近处行控制舵机；行位置、P/D 系数由受限调参头文件给出。
            int sample_row = MT9V03X_H - RACE_STEER_SAMPLE_ROW_FROM_BOTTOM;
            float mid_err;
            float steer_cmd;
            float tmp_duty;
            if (sample_row < 0) sample_row = 0;
            if (sample_row >= MT9V03X_H) sample_row = MT9V03X_H - 1;
            mid_err = (MT9V03X_W / 2.0f) - mid_line[sample_row];
            steer_cmd = RACE_STEER_KP * mid_err + RACE_STEER_KD * (mid_err - last_mid_err);
            last_mid_err = mid_err;
            tmp_duty = servo_motor_duty_middle - steer_cmd * SERVO_DIR;
            // 限幅
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));

            // 大偏差时自动降速，只能在安全上限内由调参器改值。
            speed_target = (fabsf(mid_err) >= RACE_CURVE_ERROR_THRESHOLD_PX)
                         ? RACE_SPEED_TARGET_CURVE
                         : RACE_SPEED_TARGET_STRAIGHT;

#if RACE_TELEMETRY_ENABLE
            if (++telemetry_frame >= RACE_TELEMETRY_FRAME_PERIOD)
            {
                telemetry_frame = 0;
                printf("@RACE,thr=%u,row=%d,err=%.2f,servo=%.2f,target=%.2f,enc_l=%d,enc_r=%d\r\n",
                       threshold, sample_row, mid_err, tmp_duty, speed_target,
                       encoder_data_1, encoder_data_2);
            }
#else
            (void)telemetry_frame;
#endif

            // 显示关键信息
            tft180_show_int(0, 0, encoder_data_1, 3);
            tft180_show_int(0, 30, encoder_data_2, 3);
            tft180_show_float(0, 50, speed_pwm, 4, 2);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
    }
}
