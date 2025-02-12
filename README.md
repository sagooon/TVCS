### Thrust Vector Control with PID Implementation
### **1. Header Includes**
```cpp
#include <chrono>
#include <cmath>
#include <PWMServo.h>
```
- **`#include <chrono>`**: Provides high-resolution timing functions for precise control loop timing.
- **`#include <cmath>`**: Includes mathematical functions like `std::sqrt`, `std::tan`, and `std::abs`.
- **`#include <PWMServo.h>`**: Library for controlling servos with precise PWM signals.

## 

### **2. Bessel Filter Class**
#### **Purpose**: Implements a 2nd-order Bessel low-pass filter to smooth noisy sensor data.

```cpp
class BesselFilter {
private:
    double a0, a1, a2, b1, b2; // Filter coefficients
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0; // Filter state variables
    const double fs = 500; // Sampling frequency (Hz)
```
- **`a0, a1, a2, b1, b2`**: Coefficients for the Bessel filter.
- **`x1, x2, y1, y2`**: State variables for storing previous input/output values.
- **`fs`**: Sampling frequency (500Hz).


```cpp
public:
    BesselFilter(double cutoff) {
        const double omega = 2 * M_PI * cutoff; // Convert cutoff to radians
        const double t = 1 / (std::tan(omega / (2 * fs))); // Pre-warping
        const double t2 = t * t; // t squared
        const double sqrt3 = std::sqrt(3); // Square root of 3
        
        // Calculate filter coefficients
        a0 = 3 / (1 + sqrt3 * t + t2);
        a1 = 3 * 2 / (1 + sqrt3 * t + t2);
        a2 = 3 / (1 + sqrt3 * t + t2);
        b1 = (2 - 2 * t2) / (1 + sqrt3 * t + t2);
        b2 = (1 - sqrt3 * t + t2) / (1 + sqrt3 * t + t2);
    }
```
- **`BesselFilter(double cutoff)`**: Constructor that calculates filter coefficients based on the cutoff frequency.
- **`omega`**: Angular frequency of the cutoff.
- **`t`**: Pre-warping factor for bilinear transformation.
- **`a0, a1, a2, b1, b2`**: Coefficients derived from the Bessel filter design.


```cpp
    double filter(double input) {
        double output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1; // Shift previous input values
        x1 = input;
        y2 = y1; // Shift previous output values
        y1 = output;
        return output;
    }
};
```
- **`filter(double input)`**: Applies the filter to the input signal.
- **`output`**: Computed filtered value using the difference equation.
- **`x2 = x1; x1 = input;`**: Shift input history.
- **`y2 = y1; y1 = output;`**: Shift output history.

## 

### **3. HV93i Servo Class**
#### **Purpose**: Controls the HV93i servo motor with rate limiting and angle clamping.

```cpp
class HV93i_Servo {
private:
    PWMServo servo; // Servo object
    const int pin; // Servo control pin
    static constexpr double MIN_PULSE = 500; // Minimum PWM pulse width (μs)
    static constexpr double MAX_PULSE = 2500; // Maximum PWM pulse width (μs)
    static constexpr double MAX_RATE = 360.1; // Maximum servo speed (°/s)
    double current_angle = 90.0; // Current servo angle (neutral position)
```
- **`PWMServo servo`**: Object to control the servo.
- **`MIN_PULSE, MAX_PULSE`**: PWM pulse width range for the servo.
- **`MAX_RATE`**: Maximum angular velocity of the servo.
- **`current_angle`**: Tracks the current servo position.

```cpp
public:
    HV93i_Servo(int p) : pin(p) { 
        servo.attach(pin, MIN_PULSE, MAX_PULSE); // Attach servo to pin
        servo.writeMicroseconds(angleToPulse(90.0)); // Set to neutral
    }
```
- **`HV93i_Servo(int p)`**: Constructor initializes the servo and sets it to neutral (90°).


```cpp
    void setAngle(double target) {
        auto now = std::chrono::steady_clock::now(); // Get current time
        static auto last = now; // Store last update time
        double dt = std::chrono::duration<double>(now - last).count(); // Time delta
        
        double maxΔ = MAX_RATE * dt; // Maximum allowed angle change
        target = std::clamp(target, 60.0, 120.0); // Clamp angle to safe range
        
        // Smooth movement
        if(std::abs(target - current_angle) > maxΔ) {
            target = current_angle + std::copysign(maxΔ, target - current_angle);
        }
        
        servo.writeMicroseconds(angleToPulse(target)); // Send PWM signal
        current_angle = target; // Update current angle
        last = now; // Update last time
    }
```
- **`setAngle(double target)`**: Moves the servo to the target angle with rate limiting.
- **`std::clamp`**: Ensures the target angle stays within safe limits (60° to 120°).
- **`copysign`**: Ensures the servo moves in the correct direction.
- **`angleToPulse`**: Converts angle to PWM pulse width.


```cpp
private:
    int angleToPulse(double ang) {
        return static_cast<int>(MIN_PULSE + (ang/180.0)*(MAX_PULSE - MIN_PULSE));
    }
};
```
- **`angleToPulse(double ang)`**: Converts an angle (0°-180°) to a PWM pulse width (500-2500μs).

## 

### **4. TVC System Class**
#### **Purpose**: Implements the thrust vector control system using PID control.

```cpp
class TVC_System {
private:
    HV93i_Servo pitch_servo{2}; // Servo at 12 o'clock
    HV93i_Servo yaw_servo{3};   // Servo at 9 o'clock
    BesselFilter imu_filter{15}; // Low-pass filter for IMU data (15Hz cutoff)
    const double kP = 2.5, kI = 0.8, kD = 0.3; // PID gains
    double integral = 0, prev_error = 0; // PID state variables
    std::chrono::time_point<std::chrono::steady_clock> last; // Last update time
```
- **`pitch_servo, yaw_servo`**: Servo objects for pitch and yaw control.
- **`imu_filter`**: Filters IMU data to reduce noise.
- **`kP, kI, kD`**: PID controller gains.
- **`integral, prev_error`**: Stores integral and previous error for PID control.
- **`last`**: Tracks the last update time.


```cpp
public:
    void update(double pitch_err, double yaw_err) {
        pitch_err = imu_filter.filter(pitch_err); // Filter pitch error
        yaw_err = imu_filter.filter(yaw_err);     // Filter yaw error

        auto now = std::chrono::steady_clock::now(); // Get current time
        double dt = std::chrono::duration<double>(now - last).count(); // Time delta
        dt = std::max(dt, 1e-6); // Prevent division by zero

        integral += (pitch_err + yaw_err) * dt; // Update integral term
        double deriv = ((pitch_err - prev_error) + (yaw_err - prev_error)) / dt; // Derivative term

        double output = kP*(pitch_err + yaw_err) + kI*integral + kD*deriv; // PID output

        pitch_servo.setAngle(90 + output); // Move pitch servo
        yaw_servo.setAngle(90 - output);  // Move yaw servo (opposite phase)

        prev_error = (pitch_err + yaw_err)/2; // Update previous error
        last = now; // Update last time
    }
```
- **`update(double pitch_err, double yaw_err)`**: Updates the TVC system based on IMU errors.
- **`integral`**: Accumulates error over time (integral term).
- **`deriv`**: Computes the rate of change of error (derivative term).
- **`output`**: PID control signal.
- **`setAngle`**: Moves servos to the computed angles.


```cpp
    void reset() {
        integral = 0; // Reset integral term
        prev_error = 0; // Reset previous error
        pitch_servo.setAngle(90); // Reset pitch servo
        yaw_servo.setAngle(90);  // Reset yaw servo
    }
};
```
- **`reset()`**: Resets the PID controller and servos to neutral.
