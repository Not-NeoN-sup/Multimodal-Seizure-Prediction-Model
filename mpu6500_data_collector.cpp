#include <iostream>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <sstream>
#include <chrono>
#include <vector>
#include <queue>
#include <numeric>
#include <algorithm>
#include <complex>
#define M_PI 3.14159265358979323846
#include <fstream>
#include <deque>
using namespace std;

class kalmanFilter{
private:
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Estimated value
    float p; // Estimation error covariance
    float k; // Kalman gain
public:
    kalmanFilter(float processNoise, float measurementNoise){
        q = processNoise;
        r = measurementNoise;
        x = 0;
        p = 1;
        k = 0;
    }
    float update(float measurement){
        // Prediction update
        p += q;
        // Measurement update
        k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
        return x;
    }
};    

class mpu6500{
private:
    int fd;
    int reg_addr = 0x68;
    int gyroRange = 0;
    int accelRange = 0;
    float accelScale = 16384;
    float gyroScale = 131;
    int16_t a_x_raw, a_y_raw, a_z_raw;
    int16_t g_x_raw, g_y_raw, g_z_raw;
    float a_x_offset, a_y_offset, a_z_offset;
    float g_x_offset, g_y_offset, g_z_offset;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;
    int sampleRate;
    int8_t data[14];
    const int queueSize = 100;
    const int sampleRateHz = 100;
    kalmanFilter kf_ax{0.1f, 0.5f};
    kalmanFilter kf_ay{0.1f, 0.5f};
    kalmanFilter kf_az{0.1f, 0.5f};
    kalmanFilter kf_gx{0.1f, 0.5f};
    kalmanFilter kf_gy{0.1f, 0.5f};
    kalmanFilter kf_gz{0.1f, 0.5f};    

public:
    // Move dataPoint to public section
    struct dataPoint {
        float ax, ay, az;
        float gx, gy, gz;
        float roll, pitch;
        chrono::time_point<chrono::steady_clock> timestamp;
    };
    
    deque<dataPoint> dataQueue; // Make public
    
    mpu6500(){
        fd = open("/dev/i2c-1", O_RDWR);
        if (fd < 0) {
            cerr << "Failed to open I2C device" << endl;
            exit(1);
        }
        if (ioctl(fd, I2C_SLAVE, reg_addr) < 0) {
            cerr << "Failed to set I2C address" << endl;
            exit(1);
        }
        if(write(fd, "\x6B\x00", 2) != 2) {
            cerr << "Failed to wake up MPU6500" << endl;
            exit(1);
        }
        if(write(fd, "\x1B\x00", 2) != 2) {
            cerr << "Failed to set accelerometer range" << endl;
            exit(1);
        }
        if(write(fd, "\x1C\x00", 2) != 2) {
            cerr << "Failed to set gyroscope range" << endl;
            exit(1);
        }
        if(write(fd, "\x19\x07", 2) != 2) {
            cerr << "Failed to set sample rate" << endl;
            exit(1);
        }
    }
    
    void calibrate(){
        float a_x_sum = 0, a_y_sum = 0, a_z_sum = 0;
        float g_x_sum = 0, g_y_sum = 0, g_z_sum = 0;
        int samples = 100;
        for(int i = 0; i < samples; i++) {
            if(write(fd, "\x3B", 1) != 1) {
                cerr << "Failed to write to MPU6500" << endl;
                continue;
            }
            if(read(fd, data, 14) != 14) {
                cerr << "Failed to read data from MPU6500" << endl;
                continue;
            }
            a_x_raw = (data[0] << 8) | data[1];
            a_y_raw = (data[2] << 8) | data[3];
            a_z_raw = (data[4] << 8) | data[5];
            g_x_raw = (data[8] << 8) | data[9];
            g_y_raw = (data[10] << 8) | data[11];
            g_z_raw = (data[12] << 8) | data[13];
            
            a_x_sum += a_x_raw / accelScale;
            a_y_sum += a_y_raw / accelScale;
            a_z_sum += a_z_raw / accelScale;
            g_x_sum += g_x_raw / gyroScale;
            g_y_sum += g_y_raw / gyroScale;
            g_z_sum += g_z_raw / gyroScale;
        }
        a_x_offset = a_x_sum / samples;
        a_y_offset = a_y_sum / samples;
        a_z_offset = a_z_sum / samples;
        g_x_offset = g_x_sum / samples;
        g_y_offset = g_y_sum / samples;
        g_z_offset = g_z_sum / samples;
        cout << "Calibration complete:" << endl;
        cout << "Accelerometer offsets: " << a_x_offset << ", " << a_y_offset << ", " << a_z_offset << endl;
        cout << "Gyroscope offsets: " << g_x_offset << ", " << g_y_offset << ", " << g_z_offset << endl;
    }
    
    void readData(){
        if(write(fd, "\x3B", 1) != 1) {
            cerr << "Failed to write to MPU6500" << endl;
            return;
        }
        if(read(fd, data, 14) != 14) {
            cerr << "Failed to read data from MPU6500" << endl;
            return;
        }
        a_x_raw = (data[0] << 8) | data[1];
        a_y_raw = (data[2] << 8) | data[3];
        a_z_raw = (data[4] << 8) | data[5];
        g_x_raw = (data[8] << 8) | data[9];
        g_y_raw = (data[10] << 8) | data[11];
        g_z_raw = (data[12] << 8) | data[13];
        
        accelX = kf_ax.update((a_x_raw / accelScale) - a_x_offset);
        accelY = kf_ay.update((a_y_raw / accelScale) - a_y_offset);
        accelZ = kf_az.update((a_z_raw / accelScale) - a_z_offset);
        gyroX = kf_gx.update((g_x_raw / gyroScale) - g_x_offset);
        gyroY = kf_gy.update((g_y_raw / gyroScale) - g_y_offset);
        gyroZ = kf_gz.update((g_z_raw / gyroScale) - g_z_offset);
        if(abs(accelX)<0.5 ){
            accelX = 0.001; // Prevent noise
        }
        if(abs(accelY)<0.5 ){
            accelY = 0.001; // Prevent noise
        }
        if(abs(accelZ)<0.5 ){
            accelZ = 0.001; // Prevent noise
        }
        if(abs(gyroX)<0.5 ){
            gyroX = 0.001; // Prevent noise
        }
        if(abs(gyroY)<0.5 ){
            gyroY = 0.001; // Prevent noise
        }
        if(abs(gyroZ)<0.5 ){
            gyroZ = 0.001; // Prevent noise
        }
        // Calculate roll and pitch
        float roll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;
        float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;

        chrono::time_point<chrono::steady_clock> timestamp = chrono::steady_clock::now();
        
        dataPoint dp{accelX, accelY, accelZ, gyroX, gyroY, gyroZ, roll, pitch, timestamp};
        
        if(dataQueue.size() >= queueSize) {
            dataQueue.pop_front();
        }
        dataQueue.push_back(dp);
    }
    
    ~mpu6500() {
        close(fd);
    }    
};

void writeToCSV(const mpu6500::dataPoint& dp) {
    static ofstream file("mpu6500_log.csv", ios::app);
    if (!file.is_open()) {
        cerr << "Failed to open log file!" << endl;
        return;
    }
    static bool headerwritten = false;
    if(headerwritten == false) {
        file << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch" << endl;
        headerwritten = true;
    }
    auto ms = chrono::duration_cast<chrono::milliseconds>(dp.timestamp.time_since_epoch()).count();
    file << ms << "," << dp.ax << "," << dp.ay << "," << dp.az << ","
         << dp.gx << "," << dp.gy << "," << dp.gz << ","
         << dp.roll << "," << dp.pitch << endl;
}

int setupNetwork() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        cerr << "Failed to create socket" << endl;
        exit(1);
    }
    return sockfd;
}

void sendDataToServer(const mpu6500::dataPoint& dp, int sockfd, const struct sockaddr_in& server_addr) {
    auto ms = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "%lld,%f,%f,%f,%f,%f,%f,%f,%f\n",
             ms, dp.ax, dp.ay, dp.az, dp.gx, dp.gy, dp.gz, dp.roll, dp.pitch);
    if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        cerr << "Failed to send data to server" << endl;
    } else {
        cout << "Data sent to server: " << buffer;
    }
}

int main() {
    mpu6500 sensor;
    sensor.calibrate();
    
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8888);
    inet_pton(AF_INET, "192.168.243.106", &server_addr.sin_addr);
    
    int sockfd = setupNetwork();
    if (sockfd < 0) {
        cerr << "Failed to set up network" << endl;
        return 1;
    }
    
    while (true) {
        sensor.readData();
        if (!sensor.dataQueue.empty()) {
            mpu6500::dataPoint dp = sensor.dataQueue.back();
            writeToCSV(dp);
            sendDataToServer(dp, sockfd, server_addr);
            cout << "Data logged: " << dp.ax << ", " << dp.ay << ", " << dp.az << ", "
                 << dp.gx << ", " << dp.gy << ", " << dp.gz << ", "
                 << dp.roll << ", " << dp.pitch << endl;    
        }
        usleep(100000); // Sleep for 100ms
    }   
    return 0;
}
