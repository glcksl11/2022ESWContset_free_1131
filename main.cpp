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

#define PI 3.141592

double Dt = 0;
uint64_t Now, lastUpdate;

//I2C i2c(I2C_SDA, I2C_SCL);

///////////////////////////////////////////////////////////////////////////


//Thread optical_thread(osPriorityNormal);

Thread MPU_thread(osPriorityRealtime); //osPriorityHigh
Thread LIDAR_thread(osPriorityHigh);
//Thread Read_thread(osPriorityRealtime6);
Thread print_thread(osPriorityAboveNormal);
//Thread optical_thread(osPriorityHigh);

//Thread MPU_thread;
//Thread optical_thread;


// InterruptIn EPA(PB_7,PullUp);
// InterruptIn EPB(PB_1,PullUp);

PMW3901 flow(D11,D12,D13,PC_3);
LidarLite sensor1(PC_9, PA_8); // //PC_9, PA_8 /Define LIDAR Lite sensor 1

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

//Serial pc(USBTX,USBRX,115200);
Serial pc(PC_12,PD_2,115200);

Timer dt;
Timer dt_2;

Timer PPM_timer;
Timer remote_timer_F;
Timer remote_timer_G;
Timer remote_timer_Al;
Timer remote_timer_Y;
Timer remote_timer_R;
Timer remote_timer_P;
Timer remote_timer_E;

FastPWM motor1(PC_7);
FastPWM motor2(PB_10);
FastPWM motor3(PB_15);
FastPWM motor4(PA_9);

//FastPWM motor2_1(PB_2);
//FastPWM motor2_2(PA_11);


InterruptIn PPM(PC_8);

int PPM_i=0, PPM_init=0;
int PPM_ch[8];

uint32_t PPM_all[19], PWM_all[19];
uint32_t PPM_signal[8]={0};


float lpf_dist, past_lpf_dist;
float prev_lpf_dist;

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
float vel_x_js2;
float vel_x_temp;
float vel_y_js;
float vel_y_js2;
float _tmp=0.f;

double temp = 0;


float mpu_time;
float main_time;
//float lidar_time;
int lidar_time;
float opt_t, lidar_t;

float hold_altitude;
char hold_flag;
char Elev_flag;
char Down_flag;


double velocity_errorX, velocity_errorY, Pvelocity_termX, Ivelocity_termX, Dvelocity_termX, prev_velocityX, Pvelocity_termY, Ivelocity_termY, Dvelocity_termY, prev_velocityY;
double angle_errorX, error_pid_x, error_pid_x1, angle_errorY, error_pid_y, error_pid_y1,angle_errorZ, error_pid_z, error_pid_z1, altitude_err;
double Pangle_termX, Iangle_termX, PtermX, ItermX, DtermX, prev_DtermX, Pangle_termY, Iangle_termY, PtermY, ItermY, DtermY, prev_DtermY, Pangle_termZ, Iangle_termZ, Dangle_termZ, prev_angle_DtermZ, prev_yaw, PtermZ, ItermZ, DtermZ, prev_DtermZ;
double altitude_Pterm, altitude_Iterm, altitude_Dterm, prev_kal_dist, prev_altitude_Dterm;
double roll_output, pitch_output, yaw_output, altitude_output, velocity_outputX, velocity_outputY;
double Prev_Dvelocity_termX, Prev_Dvelocity_termY, prev_gz, lpf_gz;

int throttle1 = 0, throttle2 = 0, throttle3 = 0, throttle4 = 0;
int throttle_2_1 = 0, throttle_2_2 = 0;

int Fmode, Gear, Elev;
int yaw_flag=0, prev_yaw_flag=0, yaw_init_flag=0;
int opt_gx = 0, opt_gy = 0;


void init_sensor();


Timer Lidar_t;
void Lidar_thread_loop(){
    float Lidar_sample=15;  //ms
    uint64_t Now_L, Now_L2, Work_L;
    // float dt_L =0;
    // uint32_t Now_L;
    // uint32_t lastUpdate_L;
    
    Lidar_t.start();
    while(1){
        // Now_L=Lidar_t.read_us();
        // dt_L=(float)((Now_L - lastUpdate_L)/1000000.0f) ;
        // lastUpdate_L = Now_L;
        // mutex.lock();

        //Now_L = Lidar_t.read_us();
        Now_L = rtos::Kernel::get_ms_count();
        sensor1.refreshRange();
        raw_dist=sensor1.getRange_cm();
        real_dist=abs(raw_dist*cos(roll_rad)*cos(pitch_rad));

        lpf_dist = past_lpf_dist*0.92 + real_dist*0.08;
        past_lpf_dist = lpf_dist;

        //kal_dist = kal1.getdist(real_dist, 0.015);
        //kal_vel = kal1.getVel();     
        
        Work_L= rtos::Kernel::get_ms_count();
        
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(Lidar_sample-(Work_L-Now_L)));

        //ThisThread::sleep_for((Lidar_sample-(Work_L-Now_L)));
        lidar_time=rtos::Kernel::get_ms_count()-Now_L;
        //Work_L = Lidar_t.read_us();

        //lidar_time = Work_L - Now_L;

        // uint64_t working_Tick = Kernel::get_ms_count();
        //ThisThread::sleep_until(Kernel::get_ms_count()+((uint64_t)(Lidar_sample*1000)-(working_Tick-current_Tick)));
        //Lidar_t.reset();
        // mutex.unlock();
        // pc.printf("%llu \n",last_Tick-Now_L);
        Lidar_t.reset();
    }
}

Timer MPU_t;
void MPU_thread_loop(){
    
    float IMU_sample=3; //4ms
    uint64_t Now_M,Work_M, done_t;
    uint64_t now_lidar = 0, work_lidar = 0;
    char lidar_cnt = 0, lidar_flag = 0;
    float axx = 0, ayy = 0, azz = 0;
    //uint64_t imsi_t1,imsi_t2;
    // uint32_t lastUpdate_M;
    //MPU_t.start();
    while(1){
        //imsi_t1=MPU_t.read_us();
        Now_M=rtos::Kernel::get_ms_count();
                               // 기존 코드
        // mutex.lock();
        //Now_M=rtos::Kernel::get_ms_count();
        // dt_M=(float)((Now_M - lastUpdate_M)/1000000.0f) ;
        // lastUpdate_M = Now_M;
        //uint64_t current_Tick = Kernel::get_ms_count();
        
        //lidar_cnt++;

        mpu9250.get_data();
        roll_rad=roll*PI/180;
        pitch_rad=pitch*PI/180;
        axx = ax+sin(pitch_rad);
        ayy = ay-sin(roll_rad);
        azz = az-cos(pitch_rad)*cos(roll_rad);
        raw_acc_x= axx*cos(pitch_rad)+(azz)*sin(pitch_rad);
        raw_acc_y= axx*sin(roll_rad)*sin(pitch_rad)+ayy*cos(roll_rad)-(azz)*sin(roll_rad)*cos(pitch_rad);
        raw_acc_z= -axx*cos(roll_rad)*sin(pitch_rad) + ayy*sin(roll_rad)+(azz)*cos(roll_rad)*cos(pitch_rad);


        //raw_acc_x= ax+sin(pitch_rad)*cos(pitch_rad);
        //raw_acc_y= ay-sin(roll_rad)*cos(roll_rad);
        //raw_acc_z= -(az/(cos(roll_rad)*cos(pitch_rad))-1)*9.81;

        /*
        if(!lidar_flag)
        {
            lidar_flag = 1;
            now_lidar = MPU_t.read_ms();
        }

        if(lidar_cnt == 5)
        {
            sensor1.refreshRangeVelocity();
            raw_dist=sensor1.getRange_cm();
            real_dist=abs(raw_dist*cos(roll_rad)*cos(pitch_rad));
            kal_dist = kal1.getdist(real_dist, deltat);
            kal_vel = kal1.getVel();

            work_lidar = MPU_t.read_ms();
            lidar_t = work_lidar - now_lidar;
            
            MPU_t.reset();
            lidar_flag = 0;
            lidar_cnt = 0;
        }*/


                                        
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
        //ThisThread::sleep_for((IMU_sample-(Work_M-Now_M)));
        mpu_time=rtos::Kernel::get_ms_count()-Now_M;
        
        
        // if(mpu_time<=3) led=1;
        // else led=0;

        // uint64_t working_Tick = Kernel::get_ms_count();
        //ThisThread::sleep_until(Kernel::get_ms_count()+((IMU_sample*1000)-(working_Tick-current_Tick)));
        //uint64_t last_Tick = Kernel::get_ms_count();
        // mutex.unlock();
        // pc.printf("  IMU\n\r");
        //pc.printf("Total time M: %llu\n", working_Tick - current_Tick);
        // imsi_t2=MPU_t.read_us();
        // temp_imsi=imsi_t2-imsi_t1;
        // MPU_t.reset();
        
    }
}

//Timer optical_t;
/*
float opt_time;
uint64_t Now_V, Work_V, END_V, Last_V;
void optical_thread_loop(){
    float optical_sample=100; //ms
    
    //optical_t.start();
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
        //optical_t.reset();
    }

}
*/
volatile float target_yaw,target_yaw_rate, target_throttle, target_roll, target_pitch;
volatile float target_Vx = 0, target_Vy = 0;
char target_yaw_test_flag=0;


float print_time;

double angle_errorZ_temp;

//Timer print_t;
void print_thread_loop(){
    float print_sample = 45;
    uint64_t Now_P, Work_P;
    //print_t.start();
    while(1){
        Now_P = rtos::Kernel::get_ms_count();
        pc.printf("%.1f %.1f\n\r", target_Vy*100, kalval_hsY[1]*100);
        //pc.printf("%.1f %.1f\n\r", target_roll, roll);
        //pc.printf("%.1f %.1f\n\r", target_Vy*100, kalval_hsY[1]*100);
        //pc.printf("%.1f %.1f\n\r", vel_y_js*100, vel_y_js2*100);
        //pc.printf("%.1f  %.1f\n\r",kalval_hsX[1]*100, kalval_hsY[1]*100);
        //pc.printf("%.1f %.1f %.1f\n\r", target_pitch, target_Vy, target_Vx);
        //pc.printf("%.1f  %.1f  %.1f\n\r", yaw, target_yaw, angle_errorZ_temp);
        //pc.printf("Start = %d FMode = %d Gear = %d  Yaw = %f Al = %f Roll = %f Pitch = %f Elev = %d \n\r",PPM_ch[0],Fmode,Gear,target_yaw,target_throttle,target_Vx,target_Vy,Elev);
        //pc.printf("%.1f %.1f\n\r",target_Vy*100,kalval_hsY[1]*100);

        Work_P = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(print_sample-(Work_P-Now_P)));
        print_time = rtos::Kernel::get_ms_count()-Now_P;
        //print_t.reset();
    }
}


////////////////

double map(double Tx, double Tin_min, double Tin_max, double Tout_min,  double Tout_max)
{
    return (Tx - Tin_min) * (Tout_max - Tout_min) / (Tin_max - Tin_min) + Tout_min;
}


void Check_CH(){
    for(int k=0;k<8;k++){
            if(PPM_ch[k]<700)PPM_ch[k]=700;
            if(PPM_ch[k]>1500)PPM_ch[k]=1500;
        }
    
    for(int i =0;i<8;i++){
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
         Gear = 1;          /// Angle Mode
        // drone.target_xpos += 0.05;
    }
    else if(PPM_ch[2]>1450 && PPM_ch[2]<=1500 && remote_timer_G.read_ms()>100){
         remote_timer_G.reset();
         Gear = 2;         /// Velocity Mode
        // drone.target_xpos -= 0.05;
    }
    ////////////CH3 GEAR END//////////

    ////////////CH4 YAW START//////////
    // if(PPM_ch[3]>=700 && PPM_ch[3]<=1080 && remote_timer_Y.read_ms()>100){
    //      remote_timer_Y.reset();
    //      target_yaw -=0.1;
            
    // }
    // else if(PPM_ch[3]>1120 && PPM_ch[3]<=1500 && remote_timer_Y.read_ms()>100){
    //      remote_timer_Y.reset();
    //      target_yaw +=0.1;
                  
    // }

    if(PPM_ch[3]>=700 && PPM_ch[3]<=1500 && remote_timer_Al.read_ms()>100){
         remote_timer_Y.reset();
         //temp=PPM_ch[3];
        //  if(PPM_ch[3]>1050&&PPM_ch[3]<1150) {yaw_flag=0;}
        //if(PPM_ch[3]<=1050||PPM_ch[3]>=1150) {
        // target_yaw -= map(PPM_ch[3],700,1500,-1,1);   //
        //}

        if(PPM_ch[3]<=1050||PPM_ch[3]>=1150)
        {
            target_yaw_rate = map(PPM_ch[3],700,1500,15,-15);
        }
        else {
            target_yaw_rate = 3.1;//3.7
        }

        // if(target_yaw > 180)
        //     target_yaw = target_yaw - 360;

        // else if(target_yaw < -180)
        //     target_yaw = target_yaw + 360;
    }

    ////////////CH4 YAW END//////////



    ////////////CH5 Altitude START//////////
    if(PPM_ch[4]>=700 && PPM_ch[4]<=1500 && remote_timer_Al.read_ms()>100){
         remote_timer_Al.reset();
         //temp=PPM_ch[4];
         target_throttle = map(PPM_ch[4],700,1500,1000,2000);   /// 일단은 쓰로틀 1000~2000으로 설정
    }
    ////////////CH5 Altitude END//////////


    ////////////CH6 Roll START//////////
    if(PPM_ch[5]>=700 && PPM_ch[5]<=1500 && remote_timer_R.read_ms()>100){
         remote_timer_R.reset();
        //  if(Gear == 1)
        //     target_roll=map(PPM_ch[5],700,1500,-10,10); //-15, 15
        //  else if(Gear == 2){
        //     if(PPM_ch[5]<=1050||PPM_ch[5]>=1150)
        //         target_Vx = map(PPM_ch[5],700,1500,-0.6,0.6);
        //     else
        //      target_Vx=0;
        //  }          
            if(PPM_ch[5]<=1050||PPM_ch[5]>=1150)
            {
                target_Vx = map(PPM_ch[5],700,1500,-1.0, 1.0);
                //target_roll = map(PPM_ch[5],700,1500,-5.0, 5.0);                
            }

            else
            {
                target_Vx=0;
                //target_roll = 0;
            }

    }
    
        ////////////CH6 Roll END//////////

    ////////////CH7 PITCH START//////////////
    if(PPM_ch[6]>=700 && PPM_ch[6]<=1500 && remote_timer_P.read_ms()>100){
         remote_timer_P.reset();
        //  if(Gear == 1)
        //     target_pitch=map(PPM_ch[6],700,1500,-5,5); //-10, 10

        //  else if(Gear == 2){
        //     if(PPM_ch[6]<=1050||PPM_ch[6]>=1150)
        //         target_Vy = map(PPM_ch[6],700,1500,-0.6,0.6);     
        //     else
        //      target_Vy=0;
        //  }
            if(PPM_ch[6]<=1050||PPM_ch[6]>=1150)
            {
                target_Vy = map(PPM_ch[6],700,1500,-1.5,1.5);
                //target_pitch = map(PPM_ch[6],700,1500,-4,4);; 
            }

            else
            {
                target_Vy=0;
                //target_pitch = 0;
            }                 
    }


    if(PPM_ch[7]>=700 && PPM_ch[7]<=750 && remote_timer_E.read_ms()>100){
        remote_timer_E.reset();
        Elev= 1;
    }
    else if(PPM_ch[7]>1450 && PPM_ch[7]<=1500 && remote_timer_E.read_ms()>100){
         remote_timer_E.reset();
         Elev = 2;         /// Velocity Mode
        // drone.target_xpos -= 0.05;
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
        for(int k=0;k<8;k++){
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


double r_out;

int main()
{
    init_sensor();
    //sensor1.Lidar_init();

    pc.baud(115200);
    dt.start();
    dt_2.start();

    motor1.pulsewidth_us(1000);
    motor2.pulsewidth_us(1000);
    motor3.pulsewidth_us(1000);
    motor4.pulsewidth_us(1000);
    
    //motor2_1.pulsewidth_us(1000);
    //motor2_2.pulsewidth_us(1000);

    wait_ms(2000);

    osThreadSetPriority(osThreadGetId(), osPriorityRealtime7);
    
    MPU_thread.start(&MPU_thread_loop);
    LIDAR_thread.start(&Lidar_thread_loop);
    print_thread.start(&print_thread_loop);
    //optical_thread.start(&optical_thread_loop);

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
    remote_timer_E.start();

    PPM.rise(&PPM_Rise);
    PPM.fall(&PPM_Fall);
    
    

    uint64_t lastUpdate = 0, firstUpdate = 0, Now = 0, Work=0;
    uint64_t now_opt = 0, work_opt = 0;

    //float distance;
    float err_roll;
    //float main_time;
    //float opt_t, lidar_t;

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
        
        if(opt_cnt == 4)
        {
            opt_gx = gx;
            opt_gy = gy;
            flow.read();
            vel_x_js2=(lpf_dist/100)*(42.*PI/180)*((float)flow.px)/(0.01*300);
            vel_y_js2=(lpf_dist/100)*(42.*PI/180)*((float)flow.py)/(0.01*300);

            vel_x_js=(lpf_dist/100)*(42.*PI/180)*((float)flow.px)/(0.01*300)+(lpf_dist/100)*(gx*(PI/180))*1.4;
            vel_y_js=(lpf_dist/100)*(42.*PI/180)*((float)flow.py)/(0.01*300)+(lpf_dist/100)*(gy*(PI/180))*1.4;            

            //vel_x_js=(lpf_dist/100)*(42.*PI/180)*((float)flow.px)/(0.01*300)+((lpf_dist/100)*(gx*(PI/180)*1.15));
            //vel_y_js=(lpf_dist/100)*(42.*PI/180)*((float)flow.py)/(0.01*300)+((lpf_dist/100)*(gy*(PI/180)*1.15));

            //vel_x_js2=(kal_dist/100)*(42.*PI/180)*((float)flow.px)/(deltat*300);
            kal_hsX.kalmanTask_hs(vel_x_js,raw_acc_x*9.81,kalval_hsX);
            kal_hsY.kalmanTask_hs(vel_y_js,raw_acc_y*9.81,kalval_hsY);

            /////////////////////////////////////////////////////////////////////////////////////////////////

            velocity_errorX = target_Vx - kalval_hsX[1];

            Pvelocity_termX = 8.5 * velocity_errorX;//7.5//6.0
            Ivelocity_termX += velocity_errorX * (0.01) * 0.0;

            Dvelocity_termX = -((kalval_hsX[1] - prev_velocityX) / (0.01));
            Dvelocity_termX = Prev_Dvelocity_termX*0.88 + Dvelocity_termX*0.12;
            Prev_Dvelocity_termX = Dvelocity_termX;

            Dvelocity_termX *= 1.4;//1.2


            velocity_errorY = target_Vy - kalval_hsY[1];

            Pvelocity_termY = 9.5 * velocity_errorY;   //6.5//4.5
            Ivelocity_termY += velocity_errorY * (0.01) * 0.0;

            Dvelocity_termY = -((kalval_hsY[1] - prev_velocityY) / (0.01));
            Dvelocity_termY = Prev_Dvelocity_termY*0.88 + Dvelocity_termY*0.12;
            Prev_Dvelocity_termY = Dvelocity_termY;    

            Dvelocity_termY *= 1.85; //1.2//

            prev_velocityX = kalval_hsX[1];
            prev_velocityY = kalval_hsY[1];

            velocity_outputX = Pvelocity_termX + Ivelocity_termX + Dvelocity_termX;
            velocity_outputY = Pvelocity_termY + Ivelocity_termY + Dvelocity_termY;

            if(velocity_outputX > 5.0)
                velocity_outputX = 5.0;
            else if(velocity_outputX < -5.0)
                velocity_outputX = -5.0;
            
            if(velocity_outputY > 4.0) //3.5
                velocity_outputY = 4.0;
            else if(velocity_outputY < -2.0)
                velocity_outputY = -2.0;


            target_roll = velocity_outputX;
            target_pitch = velocity_outputY;         

            work_opt = dt_2.read_ms();
            opt_t = work_opt - now_opt;
            dt_2.reset();
            opt_cnt = 0;
            opt_flag = 0;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Velocity X

        

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Roll
        angle_errorX =  target_roll - roll;

        Pangle_termX = 3.6 * angle_errorX;    //2.3       5
        Iangle_termX += angle_errorX * (0.003) * 1.3;  //0.12  

        error_pid_x = Pangle_termX - gx;
            
        PtermX = error_pid_x * 2.4;     // 2
        //DtermX = -((gx - error_pid_x1) / (0.003)) * 0.002;

        DtermX = -((gx - error_pid_x1) / (0.003));
        DtermX = prev_DtermX*0.99 + DtermX*0.01;
        prev_DtermX = DtermX;

        DtermX *= 0.13; //0.13//0.02;  0.005

        ItermX += error_pid_x * (0.003) * 0.01;//0.01


        ////////////////////////////////////////////////////////////////////
        //Pitch
        angle_errorY =  target_pitch - pitch;

        Pangle_termY = 5.5 * angle_errorY;//5.0
        Iangle_termY += angle_errorY * (0.003) * 1.5; //4.0

        error_pid_y = Pangle_termY - gy;
            
        PtermY = error_pid_y * 2.5;
        
        DtermY = -((gy - error_pid_y1) / (0.003));// * 0.0045; //0.0045
        DtermY = prev_DtermY*0.99 + DtermY*0.01;
        prev_DtermY = DtermY;

        DtermY *=1.0; //1.0//0.9 //0.6

        ItermY += error_pid_y * (0.003) * 0.01;

        //////////////////////////////////////////////////////////////////////
        //Yaw

        
        //angle_errorZ = target_yaw - yaw;

        //if(angle_errorZ > 180)
            //angle_errorZ = angle_errorZ - 360;
        //else if(angle_errorZ < -180)
            //angle_errorZ = angle_errorZ + 360;

        //angle_errorZ_temp = angle_errorZ;

        //Iangle_termZ += angle_errorZ * (0.003) * 0.0; //0.05
        
        //error_pid_z = 3.7 * angle_errorZ; //3.4
        //Iangle_termZ += angle_errorZ* (0.003) *5.0;//4.0
        lpf_gz = prev_gz*0.96 + gz*0.04;
        prev_gz = lpf_gz;

        PtermZ = (target_yaw_rate - lpf_gz) * 22.5;//20/ /27
        //PtermZ = (error_pid_z - lpf_gz) * 2.1; //2.4 //2.2    
        DtermZ = -((lpf_gz - error_pid_z1) / (0.003));
        DtermZ = prev_DtermZ*0.9 + DtermZ*0.1;
        prev_DtermZ = DtermZ;

        DtermZ *=0.23;

        ItermZ += error_pid_z * (0.003) * 0.0;   //0.3      


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
        else if(ItermZ < -100)
            ItermZ = -100;

        if(Iangle_termZ > 150)
            Iangle_termZ = 150;
        else if(Iangle_termZ < -150)
            Iangle_termZ = -150;


        roll_output = PtermX + Iangle_termX + DtermX + ItermX;
        pitch_output = PtermY + Iangle_termY + DtermY + ItermY;
        //yaw_output = PtermZ + DtermZ + ItermZ + Iangle_termZ;
        yaw_output = PtermZ  + DtermZ;

        if(yaw_output > 150)
        {
            yaw_output = 150;
        }

        else if(yaw_output < -150)
        {
            yaw_output = -150;
        }
    
        //Altitude_output = altitude_Pterm + altitude_Iterm + altitude_Dterm;

        error_pid_x1 = gx;
        error_pid_y1 = gy;
        error_pid_z1 = lpf_gz;//gz
        
        ///////////////////////////////////////////////////////
        

        // throttle1 = target_throttle + (int)roll_output;
        // throttle2 = target_throttle - (int)roll_output;
        // throttle3 = target_throttle - (int)roll_output;
        // throttle4 = target_throttle + (int)roll_output;  

        ////////////////////준석 속도 PID//////////////
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
        ////////////////////준석 속도 PID//////////////
        //throttle1 = target_throttle + (int)roll_output;
        //throttle2 = target_throttle - (int)roll_output;
        //throttle3 = target_throttle - (int)roll_output;
        //throttle4 = target_throttle + (int)roll_output;

        // throttle1 = target_throttle + roll_output;
        // throttle2 = target_throttle - roll_output;
        // throttle3 = target_throttle - roll_output;
        // throttle4 = target_throttle + roll_output; 

        // throttle1 = target_throttle + (int)r_out;
        // throttle2 = target_throttle - (int)r_out;
        // throttle3 = target_throttle - (int)r_out;
        // throttle4 = target_throttle + (int)r_out;  

        if(Fmode == 1)
        {
            roll_output = 0;
            pitch_output = 0;
            yaw_output = 0;
            Iangle_termX = 0;
            Iangle_termY = 0;
            Iangle_termZ = 0;

            ItermX = 0;
            ItermY = 0;
            ItermZ = 0;
            altitude_Iterm = 0;

            yaw_init_flag=0;

            // throttle1 = (int)(target_throttle);
            // throttle2 = (int)(target_throttle);
            // throttle3 = (int)(target_throttle);
            // throttle4 = (int)(target_throttle);

            throttle1 = (int)(1000);
            throttle2 = (int)(1000);
            throttle3 = (int)(1000);
            throttle4 = (int)(1000);
            //throttle_2_1 = (int)(target_throttle);
            //throttle_2_2 = (int)(target_throttle);

        }

        else if(Fmode == 2) // altitude pid mode
        {
            if(!yaw_init_flag)
            {
                target_yaw = yaw;
                yaw_init_flag=1;
            }

            throttle1 = (int)(30 + target_throttle + (int)roll_output - (int)pitch_output + (int)yaw_output);
            throttle2 = (int)(30 + target_throttle - (int)roll_output - (int)pitch_output - (int)yaw_output);
            throttle3 = (int)( target_throttle - (int)roll_output + (int)pitch_output + (int)yaw_output);
            throttle4 = (int)( target_throttle + (int)roll_output + (int)pitch_output - (int)yaw_output);

            hold_flag = 0;

        }

        else if(Fmode == 3)
        {
            if(hold_flag == 0)
            {
               //hold_altitude = kal_dist;
               hold_altitude = lpf_dist;
               hold_flag = 1;               
            }
            /////// 고도 +///////
            if(Elev == 1)
                Elev_flag=0;
            if(Elev==2){
                if(Elev_flag==0)
                {
                    hold_altitude+=20;
                    Elev_flag=1;
                    if(hold_altitude >= 200)
                        hold_altitude = 200;
                }   
            }
            //////////////////////
            /////// 고도 -///////
            if(Gear == 1)
                Down_flag=0;
            if(Gear==2){
                if(Down_flag==0){
                    hold_altitude-=20;
                    Down_flag=1;
                    //if(hold_altitude <= 50)
                    //    hold_altitude = 50;
                }   
            }

            altitude_err = (hold_altitude - lpf_dist) / 100.0;

            altitude_Pterm = altitude_err * 40.0; //90.0
            altitude_Iterm += altitude_err * 65.0 * 0.003; //80//40.0

            altitude_Dterm = -(lpf_dist - prev_lpf_dist) / 0.003;
            altitude_Dterm = prev_altitude_Dterm*0.87 + altitude_Dterm*0.13;//0.99,0.01
            prev_altitude_Dterm = altitude_Dterm;

            altitude_Dterm*= 2.0; //0.8  1.3

            if(altitude_Iterm > 300)
                altitude_Iterm = 300;
            else if(altitude_Iterm < -300)
                altitude_Iterm = -300;            

            altitude_output = altitude_Pterm + altitude_Iterm + altitude_Dterm; //+
            
            throttle1 = (int)(30 + target_throttle + (int)altitude_output + (int)roll_output - (int)pitch_output + (int)yaw_output);
            throttle2 = (int)(30 + target_throttle + (int)altitude_output - (int)roll_output - (int)pitch_output - (int)yaw_output);
            throttle3 = (int)(target_throttle + (int)altitude_output - (int)roll_output + (int)pitch_output + (int)yaw_output);
            throttle4 = (int)(target_throttle + (int)altitude_output + (int)roll_output + (int)pitch_output - (int)yaw_output);          

        }
        
        //////////////////////////////
        prev_lpf_dist = lpf_dist;
        /////////////////////////////////


        //throttle1 = (int)(80 + target_throttle + (int)roll_output - (int)pitch_output + (int)yaw_output);
        //throttle2 = (int)(80 + target_throttle - (int)roll_output - (int)pitch_output - (int)yaw_output);
        //throttle3 = (int)( target_throttle - (int)roll_output + (int)pitch_output + (int)yaw_output);
        //throttle4 = (int)( target_throttle + (int)roll_output + (int)pitch_output - (int)yaw_output);

        //throttle1 = target_throttle + (int)(+roll_output);
        //throttle2 = target_throttle + (int)(-roll_output);
        //throttle3 = target_throttle + (int)(-roll_output);
        //throttle4 = target_throttle + (int)(+roll_output);

        // throttle1 = 50 + target_throttle + (int)(-pitch_output);
        // throttle2 = 50 + target_throttle + (int)(-pitch_output);
        // throttle3 = target_throttle + (int)(+pitch_output);
        // throttle4 = target_throttle + (int)(+pitch_output);        

        //throttle1 = target_throttle + (int)(+yaw_output);
        //throttle2 = target_throttle + (int)(-yaw_output);
        //throttle3 = target_throttle + (int)(+yaw_output);
        //throttle4 = target_throttle + (int)(-yaw_output);              


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
        
       // motor2_1.pulsewidth_us(throttle_2_1);
       // motor2_2.pulsewidth_us(throttle_2_2);


       
        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(3-(Work-Now)));
        //ThisThread::sleep_for((3-(Work-Now)));
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
    //pc.printf("magcal start\n");
    //mpu9250.magcalMPU9250(magbias, magScale);
    //pc.printf("%.3f %.3f %.3f // %.3f %.3f %.3f\n",magbias[0],magbias[1],magbias[2],magScale[0],magScale[1],magScale[2]);
    flow.init();
  
}
