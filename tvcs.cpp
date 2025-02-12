#include <chrono>
#include <cmath>
#include <PWMServo.h>

class BesselFilter {
private:
    double a0, a1, a2, b1, b2;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    const double fs = 500;
public:
    BesselFilter(double cutoff) {
        const double omega = 2 * M_PI * cutoff;
        const double t = 1 / (std::tan(omega / (2 * fs)));
        const double t2 = t * t;
        const double sqrt3 = std::sqrt(3);
        
        a0 = 3 / (1 + sqrt3 * t + t2);
        a1 = 3 * 2 / (1 + sqrt3 * t + t2);
        a2 = 3 / (1 + sqrt3 * t + t2);
        b1 = (2 - 2 * t2) / (1 + sqrt3 * t + t2);
        b2 = (1 - sqrt3 * t + t2) / (1 + sqrt3 * t + t2);
    }
    double filter(double input) {
        double output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }
};

class HV93i_Servo {
private:
    PWMServo servo;
    const int pin;
    static constexpr double MIN_PULSE = 500;
    static constexpr double MAX_PULSE = 2500;
    static constexpr double MAX_RATE = 360.1;
    double current_angle = 90.0;
public:
    HV93i_Servo(int p) : pin(p) { 
        servo.attach(pin, MIN_PULSE, MAX_PULSE);
        servo.writeMicroseconds(angleToPulse(90.0));
    }
    void setAngle(double target) {
        auto now = std::chrono::steady_clock::now();
        static auto last = now;
        double dt = std::chrono::duration<double>(now - last).count();
        
        double maxΔ = MAX_RATE * dt;
        target = std::clamp(target, 60.0, 120.0);
        
        if(std::abs(target - current_angle) > maxΔ) {
            target = current_angle + std::copysign(maxΔ, target - current_angle);
        }
        
        servo.writeMicroseconds(angleToPulse(target));
        current_angle = target;
        last = now;
    }
private:
    int angleToPulse(double ang) {
        return static_cast<int>(MIN_PULSE + (ang/180.0)*(MAX_PULSE - MIN_PULSE));
    }
};

class TVC_System {
private:
    HV93i_Servo pitch_servo{2};
    HV93i_Servo yaw_servo{3};
    BesselFilter imu_filter{15};
    const double kP = 2.5, kI = 0.8, kD = 0.3;
    double integral = 0, prev_error = 0;
    std::chrono::time_point<std::chrono::steady_clock> last;
public:
    void update(double pitch_err, double yaw_err) {
        pitch_err = imu_filter.filter(pitch_err);
        yaw_err = imu_filter.filter(yaw_err);
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        dt = std::max(dt, 1e-6);
        integral += (pitch_err + yaw_err) * dt;
        double deriv = ((pitch_err - prev_error) + (yaw_err - prev_error)) / dt;
        double output = kP*(pitch_err + yaw_err) + kI*integral + kD*deriv;
        pitch_servo.setAngle(90 + output);
        yaw_servo.setAngle(90 - output);
        prev_error = (pitch_err + yaw_err)/2;
        last = now;
    }
    void reset() {
        integral = 0;
        prev_error = 0;
        pitch_servo.setAngle(90);
        yaw_servo.setAngle(90);
    }
};
