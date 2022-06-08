#include "mbed.h"
#include "LidarLite.h"
#include "MPU9250.h"
#include <cstdint>
#include "kalman.h"
#include "kalmanhs.h"
#include "Thread.h"
#include "rtos.h"
//#include "kalmanacc.h"
#include "Matrix.h"
#include "PMW3901.h"
#include "PID.h"
#include "FastPWM.h"
#define LIDARLite1_SDA I2C_SDA   //SDA pin on LPC1768
#define LIDARLite1_SCL I2C_SCL  //SCL pin on LPC1768

#define PI 3.141592

double Dt = 0;
uint64_t Now, lastUpdate;

//I2C i2c(I2C_SDA, I2C_SCL);

///////////////////////////////////////////////////////////////////////////


//Thread optical_thread(osPriorityNormal);

Thread MPU_thread(osPriorityHigh); //osPriorityHigh
//Thread Read_thread(osPriorityRealtime6);
Thread print_thread(osPriorityNormal);

//Thread LIDAR_thread(osPriorityAboveNormal);
//Thread optical_thread(osPriorityHigh);

//Thread MPU_thread;
//Thread optical_thread;


// InterruptIn EPA(PB_7,PullUp);
// InterruptIn EPB(PB_1,PullUp);

PMW3901 flow(D11,D12,D13,PC_3);
LidarLite sensor1(PB_9, PB_8); //Define LIDAR Lite sensor 1

MPU9250 mpu9250;
kalman kal1, kal2;
kalmanhs kal_hsX,kal_hsY;

//////////////////////////// PID (kp,ki,kd,imax) 순임 
PID PID_R_velocity(1,0,0,100);
PID PID_R_angle(5, 0, 0, 100);
PID PID_R_rate(1.5, 0, 0, 150);//i=0.1
PID PID_P_angle(3.5,0.1,0,100);
PID PID_P_rate(2.1,0.1,0.0045,150);
//////////////////////////////
//kalmanacc kalacc;

Serial pc(USBTX,USBRX,115200);

Timer dt;
Timer dt_2;
Timer PPM_timer;
Timer remote_timer_F;
Timer remote_timer_G;
Timer remote_timer_Al;
Timer remote_timer_Y;
Timer remote_timer_R;
Timer remote_timer_P;

FastPWM motor1(PC_7);
FastPWM motor2(PB_10);
FastPWM motor3(PA_8);
FastPWM motor4(PA_9);

InterruptIn PPM(PC_8);

int PPM_i=0, PPM_init=0;
int PPM_ch[7];

uint32_t PPM_all[19], PWM_all[19];
uint32_t PPM_signal[7]={0};


float raw_dist=0;
float real_dist=0;
float kal_dist=0;
float kal2_dist=0;
float past_dist=0;
float kal_vel=0;
float kal_acc=0;
float raw_acc_x=0;
float raw_acc_y=0;
float raw_acc_z=0;
float roll_rad = 0 , pitch_rad = 0;
int ct=0;
float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
float kal_val[3]={0,0,0};
float kalval_hsX[3]={0,0,0};
float kalval_hsY[3]={0,0,0};
float vel_x_hs;
float vel_x_hs2;
float vel_x_js;
float vel_y_js;
float _tmp=0.f;

double temp = 0;

void init_sensor();

/*
Timer Lidar_t;
void Lidar_thread_loop(){
    float Lidar_sample=3;  //ms
    // float dt_L =0;
    // uint32_t Now_L;
    // uint32_t lastUpdate_L;
    
    while(1){
        // Now_L=Lidar_t.read_us();
        // dt_L=(float)((Now_L - lastUpdate_L)/1000000.0f) ;
        // lastUpdate_L = Now_L;
        // mutex.lock();
        uint64_t Now_L = Kernel::get_ms_count();
     
        uint64_t Work_L= Kernel::get_ms_count();
        ThisThread::sleep_until(Kernel::get_ms_count()+(Lidar_sample-(Work_L-Now_L)));

        // uint64_t working_Tick = Kernel::get_ms_count();
        // ThisThread::sleep_until(Kernel::get_ms_count()+((uint64_t)(Lidar_sample*1000)-(working_Tick-current_Tick)));
         uint64_t last_Tick = Kernel::get_ms_count();
        //Lidar_t.reset();
        // mutex.unlock();
        // pc.printf("%llu \n",last_Tick-Now_L);
        Lidar_t.reset();
    }
}*/

float read_time;
Timer read_t;
void read_thread_loop(){
    float read_sample = 3;
    uint64_t Now_R, Work_R;
    read_t.start();
    while(1){
        Now_R = rtos::Kernel::get_ms_count();

        mpu9250.read_data();
        roll_rad=roll*PI/180;
        pitch_rad=pitch*PI/180;
        raw_acc_x= ax+sin(pitch_rad)*cos(pitch_rad);
        raw_acc_y= ay-sin(roll_rad)*cos(roll_rad);
        raw_acc_z= -(az/(cos(roll_rad)*cos(pitch_rad))-1)*9.81;

        Work_R = rtos::Kernel::get_ms_count();

        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(read_sample-(Work_R-Now_R)));
        read_time = rtos::Kernel::get_ms_count()-Now_R;
    }
}



float mpu_time;
float main_time;

Timer MPU_t;
void MPU_thread_loop(){
    
    float IMU_sample=3; //4ms
     uint64_t Now_M,Work_M,done_t;
    // uint32_t lastUpdate_M;
    MPU_t.start();
    while(1){
         Now_M=rtos::Kernel::get_ms_count();
                               // 기존 코드
       // mutex.lock();
         //Now_M=rtos::Kernel::get_ms_count();
        // dt_M=(float)((Now_M - lastUpdate_M)/1000000.0f) ;
        // lastUpdate_M = Now_M;
        //uint64_t current_Tick = Kernel::get_ms_count();

        mpu9250.get_data();
      

        // sensor1.refreshRangeVelocity();
        // raw_dist=sensor1.getRange_cm();
        // real_dist=abs(raw_dist*cos(roll_rad)*cos(pitch_rad));
        // kal_dist = kal1.getdist(real_dist, deltat);
        // kal_vel = kal1.getVel();
                                        
        // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
        // yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        // pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        // roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        // pitch *= 180.0f / PI;
        // yaw   *= 180.0f / PI; 
        // //yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        // roll  *= 180.0f / PI;


        Work_M=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(IMU_sample-(Work_M-Now_M)));
        mpu_time=rtos::Kernel::get_ms_count()-Now_M;
        
        
        // if(mpu_time<=3) led=1;
        // else led=0;

        // uint64_t working_Tick = Kernel::get_ms_count();
        //ThisThread::sleep_until(Kernel::get_ms_count()+((IMU_sample*1000)-(working_Tick-current_Tick)));
        //uint64_t last_Tick = Kernel::get_ms_count();
        // mutex.unlock();
        // pc.printf("  IMU\n\r");
        //pc.printf("Total time M: %llu\n", working_Tick - current_Tick);
        MPU_t.reset();
        
    }
}

Timer optical_t;
float opt_time;
uint64_t Now_V,Work_V,END_V,Last_V;
void optical_thread_loop(){
    float optical_sample=100; //ms
    
    optical_t.start();
    while(1){
        
        //mutex.lock();
        Now_V=rtos::Kernel::get_ms_count();
        
        
        flow.read();
        vel_x_js=(kal_dist/100)*(42.*PI/180)*((float)flow.px)/(deltat*300)-(kal_dist/100)*(gx*(PI/180));
        vel_y_js=(kal_dist/100)*(42.*PI/180)*((float)flow.py)/(deltat*300)-(kal_dist/100)*(gy*(PI/180));
        //vel_x_js2=(kal_dist/100)*(42.*PI/180)*((float)flow.px)/(deltat*300);
        kal_hsX.kalmanTask_hs(vel_x_js,raw_acc_x*9.81,kalval_hsX);
        kal_hsY.kalmanTask_hs(vel_y_js,raw_acc_y*9.81,kalval_hsY);
        Work_V= rtos::Kernel::get_ms_count();
        
        //pc.printf("%.1f\n\r", opt_time);
        // while(Kernel::get_ms_count()-Now_V<100){
            
        // }
        // opt_time=Kernel::get_ms_count()-Now_V;
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(optical_sample-(Work_V-Now_V)));
        opt_time=rtos::Kernel::get_ms_count()-Now_V;
        //kalhs_2.kalmanTask_hs(vel_x_js2, raw_acc_y*9.81,kalval_hs2);
        //mutex.unlock();
        //pc.printf("Optical\n\r");
        optical_t.reset();
    }

}
volatile float target_yaw, target_altitude, target_roll, target_pitch;
volatile float target_Vx, target_Vy=0;


float print_time;
Timer print_t;
void print_thread_loop(){
    float print_sample = 10;
    uint64_t Now_P, Work_P;
    print_t.start();
    while(1){
        Now_P = rtos::Kernel::get_ms_count();
        pc.printf("%.3f %.3f\n\r", target_roll,roll);
        Work_P = rtos::Kernel::get_ms_count();
        //ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(print_sample-(Work_P-Now_P)));
        print_time= rtos::Kernel::get_ms_count()-Now_P;
        print_t.reset();
    }
}


int Fmode, Gear;
////////////////




double map(double Tx, double Tin_min, double Tin_max, double Tout_min,  double Tout_max)
{
    return (Tx - Tin_min) * (Tout_max - Tout_min) / (Tin_max - Tin_min) + Tout_min;
}


void Check_CH(){
    for(int k=0;k<7;k++){
            if(PPM_ch[k]<700)PPM_ch[k]=700;
            if(PPM_ch[k]>1500)PPM_ch[k]=1500;
        }
    
    for(int i =0;i<7;i++){
            PPM_signal[i]=PPM_ch[i];
        }
    //////////CH2 FMODE START /////////////
    
    if(PPM_ch[1]>=700 && PPM_ch[1]<=750 && remote_timer_F.read_ms()>100){
         remote_timer_F.reset();
         Fmode = 1;   ///// mode 1   호버링?
       
    }
    else if(PPM_ch[1]>1050 && PPM_ch[1]<=1150 && remote_timer_F.read_ms()>100){
         remote_timer_F.reset();
         Fmode = 2;   //// mode 2    조종?
        
    }
    else if(PPM_ch[1]>1450 && PPM_ch[1]<=1500 && remote_timer_F.read_ms()>100){
         remote_timer_F.reset();
         Fmode = 3;   //// mode 3    삽입?
        
    }
    ////////////CH2 FMODE END//////////


   ////////////CH3 GEAR START//////////

    if(PPM_ch[2]>=700 && PPM_ch[2]<=750 && remote_timer_G.read_ms()>100){
         remote_timer_G.reset();
         Gear = 1;          /// 벽 프로펠러 온?
        // drone.target_xpos += 0.05;
    }
    else if(PPM_ch[2]>1450 && PPM_ch[2]<=1500 && remote_timer_G.read_ms()>100){
         remote_timer_G.reset();
         Gear = 2;         /// 벽 프로펠러 오프?
        // drone.target_xpos -= 0.05;
    }
    ////////////CH3 GEAR END//////////

    ////////////CH4 YAW START//////////
    if(PPM_ch[3]>=700 && PPM_ch[3]<=1080 && remote_timer_Y.read_ms()>100){
         remote_timer_Y.reset();
         target_yaw -=0.1;
            
    }
    else if(PPM_ch[3]>1120 && PPM_ch[3]<=1500 && remote_timer_Y.read_ms()>100){
         remote_timer_Y.reset();
         target_yaw +=0.1;
                  
    }
    ////////////CH4 YAW END//////////



    ////////////CH5 Altitude START//////////
    if(PPM_ch[4]>=700 && PPM_ch[4]<=1500 && remote_timer_Al.read_ms()>100){
         remote_timer_Al.reset();
         target_altitude = map(PPM_ch[4],700,1500,1000,2000);   /// 일단은 쓰로틀 1000~2000으로 설정
    }
    ////////////CH5 Altitude END//////////


    ////////////CH6 Roll START//////////
    if(PPM_ch[5]>=700 && PPM_ch[5]<=1500 && remote_timer_R.read_ms()>100){
         remote_timer_R.reset();
         target_roll=map(PPM_ch[5],700,1500,-15,15);
           
    }
    
        ////////////CH6 Roll END//////////

    ////////////CH7 PITCH START//////////////
    if(PPM_ch[6]>=700 && PPM_ch[6]<=1500 && remote_timer_P.read_ms()>100){
         remote_timer_P.reset();
         target_pitch=map(PPM_ch[6],700,1500,-10,10);
                 
    }
       ////////////CH7 PITCH START//////////////


}
void PPM_Rise(){
    PPM_timer.reset();
}
void PPM_Fall(){
    PPM_all[PPM_i] = PPM_timer.read_us();
    PPM_i++;
    if(PPM_i==17){
        for(int k=18;k>-1;k--){
            if(PPM_all[k]>10000){
                PPM_init = k;
            }
        }
        for(int k=0;k<7;k++){
            PPM_ch[k]=PPM_all[PPM_init+k+1];
        }
        PPM_i = 0;
        Check_CH();
    }
}

/////////////////////PID 코드  PID.cpp참조///////////////////////
double prev_gx=0;
double prev_angle_x=0;

double prev_gy=0;
double prev_angle_y=0;

double Roll_pid(double angle, double rate, double velocity){
    // float error_v = target_Vx-velocity;
    // float target_angle = PID_R_velocity.get_PID(error_v, velocity, deltat);
    double error = target_roll-angle;
    double target_Rate = PID_R_angle.P_control(error);
    double error_rate = target_Rate-rate;
    double result = PID_R_rate.get_PID(error_rate, rate,prev_gx, 0.005)+PID_R_angle.I_control(error, 0.004);  //PIPID
    temp = PID_R_rate.I_control(error, 0.004);
    prev_gx=rate;
    prev_angle_x=angle;
    return result;
}

float Pitch_pid(float angle, float rate){
    float error = target_pitch-angle;
    float target_Rate = PID_P_angle.P_control(error);
    float error_rate = target_Rate-rate;
    float result = PID_P_rate.get_PID(error_rate, rate,prev_gy, 0.005)+PID_P_angle.I_control(error, 0.004);  //PIPID
    prev_gy=rate;
    return result;
}



/////////////////////PID 함수 끝///////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

int en_pos = 0;
// void doEncoderA(void)
// {
//   en_pos += (EPA == EPB)?1:-1;
// }

// void doEncoderB(void)
// {
//   en_pos += (EPA == EPB)?-1:1;
// }

double angle_errorX, error_pid_x, error_pid_x1, angle_errorY, error_pid_y, error_pid_y1, error_pid_z, error_pid_z1;
double Pangle_termX, Iangle_termX, PtermX, ItermX, DtermX, Pangle_termY, Iangle_termY, PtermY, ItermY, DtermY, PtermZ, DtermZ, ItermZ;;
double roll_output, pitch_output, yaw_output;

int throttle1, throttle2, throttle3, throttle4;

double r_out;

int main()
{
    init_sensor();
    pc.baud(115200);
    dt.start();
    dt_2.start();

    motor1.pulsewidth_us(1000);
    motor2.pulsewidth_us(1000);
    motor3.pulsewidth_us(1000);
    motor4.pulsewidth_us(1000);
    MPU_thread.start(&MPU_thread_loop);
    //Read_thread.start(&read_thread_loop);
    print_thread.start(&print_thread_loop);
    //LIDAR_thread.start(&Lidar_thread_loop);
    //optical_thread.start(&optical_thread_loop);

    osThreadSetPriority(osThreadGetId(), osPriorityRealtime7);

    //MPU_thread.join();
    //LIDAR_thread.join();

    //pc.printf("init done\n\r");

    //tSensor();
    //caliSensor();

    PPM.enable_irq();
    PPM_timer.start();

    remote_timer_F.start();
    remote_timer_G.start();
    remote_timer_Al.start();
    remote_timer_Y.start();
    remote_timer_R.start();
    remote_timer_P.start();
    
    PPM.rise(&PPM_Rise);
    PPM.fall(&PPM_Fall);

    

    uint64_t lastUpdate = 0, firstUpdate = 0, Now = 0, Work=0;
    uint64_t now_opt = 0, work_opt = 0;

    //float distance;
    float err_roll;
    //float main_time;
    float opt_t;
    char opt_cnt = 0, opt_flag = 0;
    // EPA.rise(&doEncoderA);
    // EPA.fall(&doEncoderA);
    // EPB.rise(&doEncoderB);
    // EPB.fall(&doEncoderB);

    while(1)
    {
        Now = rtos::Kernel::get_ms_count();
        //main_time = (float)((Now - lastUpdate)/1000.0f); // set integration time by time elapsed since last filter update
        //lastUpdate = Now;

        
        opt_cnt++;

        if(!opt_flag)
        {
             now_opt = dt_2.read_ms();
             opt_flag = 1;
        }

        
        if(opt_cnt == 26)
        {
            flow.read();
            vel_x_js=(kal_dist/100)*(42.*PI/180)*((float)flow.px)/(main_time*300)-(kal_dist/100)*(gx*(PI/180));
            vel_y_js=(kal_dist/100)*(42.*PI/180)*((float)flow.py)/(main_time*300)-(kal_dist/100)*(gy*(PI/180));
            //vel_x_js2=(kal_dist/100)*(42.*PI/180)*((float)flow.px)/(deltat*300);
            kal_hsX.kalmanTask_hs(vel_x_js,raw_acc_x*9.81,kalval_hsX);
            kal_hsY.kalmanTask_hs(vel_y_js,raw_acc_y*9.81,kalval_hsY);

            work_opt = dt_2.read_ms();
            opt_t = work_opt - now_opt;
            dt_2.reset();
            opt_cnt = 0;
            opt_flag = 0;
        }
            
        //////////////////////////////////////////////////////////
        //roll
        angle_errorX =  target_roll - roll;

        Pangle_termX = 3.6 * angle_errorX;    //2.3
        Iangle_termX += angle_errorX * (0.003) * 1.3;  //0.12

        error_pid_x = Pangle_termX - gx;
            
        PtermX = error_pid_x * 2.4;
        DtermX = -((gx - error_pid_x1) / (0.003)) * 0.002;
        ItermX += error_pid_x * (0.003) * 0.01;//0.5

        ////////////////////////////////////////////////////////////////////
        //pitch
        angle_errorY =  target_pitch - pitch;

        Pangle_termY = 3.5 * angle_errorY;//3.5
        Iangle_termY += angle_errorY * (0.003) * 0.1;

        error_pid_y = Pangle_termY - gy;
            
        PtermY = error_pid_y * 2.1;
        DtermY = -((gy - error_pid_y1) / (0.003)) * 0.0045; //0.0045
        ItermY += error_pid_y * (0.003) * 0.1;

        ////////////////////////////////////////////////////////////////////
        //yaw
        error_pid_z = target_yaw - gz;

        PtermZ = error_pid_z * 2.5;
        DtermZ = -((gz - error_pid_z1) / (0.003)) * 0.1;
        ItermZ += error_pid_z * (0.003) * 0.7;
        
        if(Iangle_termX > 100)
            Iangle_termX = 100;
        else if( Iangle_termX < -100)
            Iangle_termX = -100;

        if(ItermX > 150)
            ItermX = 150;
        else if(ItermX < -150)
            ItermX = -150;
        /////////////////////////////////////////

        if(Iangle_termY > 150)
            Iangle_termY = 150;
        else if( Iangle_termY < -150)
            Iangle_termY = -150;

        if(ItermY > 150)
            ItermY = 150;
        else if(ItermY < -150)
            ItermY = -150;

        if(ItermZ > 100)
            ItermZ = 100;
        else if( ItermZ < -100)
            ItermZ = -100;

        roll_output = PtermX + Iangle_termX + DtermX + ItermX;
        pitch_output = PtermY + Iangle_termY + DtermY + ItermY;
        yaw_output = PtermZ + DtermZ + ItermZ;

        error_pid_x1 = gx;
        error_pid_y1 = gy;
        error_pid_z1 = gz;
        
        ///////////////////////////////////////////////////////
        /*
        if(Fmode == 2)
        {
            r_out=Roll_pid(roll, gx,kalval_hsX[1]);            
        }

        else 
        {
            r_out = 0;
            PID_R_angle.reset();
            PID_R_rate.reset();
        }
        */

        throttle1 = target_altitude + (int)(+roll_output - pitch_output + yaw_output;
        throttle2 = target_altitude + (int)(-roll_output - pitch_output - yaw_output);
        throttle3 = target_altitude + (int)(-roll_output + pitch_output + yaw_output);
        throttle4 = target_altitude + (int)(+roll_output + pitch_output - yaw_output);        


        if(throttle1 > 2000)    throttle1 = 2000;   //200넘으면 200으로
        else if(throttle1 <= 1000)  throttle1 = 1000;   //110보다 작아지면 110으로
        if(throttle2 > 2000)    throttle2 = 2000;
        else if(throttle2 <= 1000)  throttle2 = 1000;
        if(throttle3 > 2000)    throttle3 = 2000;
        else if(throttle3 <= 1000)  throttle3 = 1000;
        if(throttle4 > 2000)    throttle4 = 2000;
        else if(throttle4 <= 1000)  throttle4 = 1000;    

        motor1.pulsewidth_us(throttle1); //pitch 추가     1    2
        motor2.pulsewidth_us(throttle2); //pitch 추가     4    3   모터로 생각
        motor3.pulsewidth_us(throttle3); //pitch 추가     변수이름은 altitude이지만
        motor4.pulsewidth_us(throttle4); //pitch 추가     지금은 throttle
        
        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(3-(Work-Now)));
        main_time = (float)((rtos::Kernel::get_ms_count() - Now)/1000.0f);
        dt.reset();
    }
}


void init_sensor(){
    pc.baud(115200);
    //dt.start();
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    pc.printf("I AM 0x%x\t", whoami); pc.printf("I SHOULD BE 0x71\n\r");
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values 
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    //ThisThread::sleep_for(100);
    mpu9250.initMPU9250(); 
    mpu9250.initAK8963(magCalibration);
    //whoami = mpu9250.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    //pc.printf("I AM 0x%x\t", whoami); pc.printf("I SHOULD BE 0x48\n\r");
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
   // ThisThread::sleep_for(100);
    flow.init();
    pc.printf("Init Done\n\r");
}
