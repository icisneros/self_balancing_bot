#include "compat/Arduino.h"
#include <array>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

// Arduino normally generates these declarations before compiling the sketch.
void init_IO(); void i2cSetup(); void MPU6050Connect(); void pid_setup();
void motor_shield_setup(); void GetDMP(); void pid_control();
void control_left(); void control_right(); void MPUMath();
#include "../DCMotor_and_IMU/DCMotor_and_IMU.ino"

namespace {
constexpr double rad = M_PI / 180.0;
using State = std::array<double, 4>; // theta, angular velocity, axle x, axle velocity
struct Plant {
    double body_mass, axle_mass, com_x, length, inertia, gravity, wheel_radius, drag, damping;
    State derivative(const State& s, double force) const {
        double sn = std::sin(s[0]), cs = std::cos(s[0]);
        double horizontal = com_x*cs + length*sn;
        double vertical = length*cs - com_x*sn;
        double a = body_mass + axle_mass, b = body_mass * vertical;
        double d = inertia + body_mass*(length*length + com_x*com_x);
        double r1 = force - drag*s[3] + body_mass*horizontal*s[1]*s[1];
        // Equal/opposite wheel-drive torque acts on the body.
        double r2 = body_mass*gravity*horizontal - damping*s[1] - force*wheel_radius;
        double det = a*d - b*b;
        return {s[1], (a*r2 - b*r1)/det, s[3], (d*r1 - b*r2)/det};
    }
    void step(State& s, double force, double dt) const {
        auto shift = [](State x, State k, double h) {
            for (int i=0; i<4; ++i) x[i] += h*k[i];
            return x;
        };
        State k1 = derivative(s, force), k2 = derivative(shift(s, k1, dt/2), force);
        State k3 = derivative(shift(s, k2, dt/2), force), k4 = derivative(shift(s, k3, dt), force);
        for (int i=0; i<4; ++i) s[i] += dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
    }
};
}

int main(int argc, char** argv) {
    try {
        std::map<std::string, double> c;
        for (int i=1; i+1<argc; i+=2) c[argv[i]] = std::stod(argv[i+1]);
        auto v = [&](const std::string& name, double fallback) {
            return c.count(name) ? c.at(name) : fallback;
        };
        const double duration=v("duration", 12), dt=0.0005;
        if (!(duration > 0 && duration <= 30)) throw std::runtime_error("duration must be in (0, 30]");
        const double initial=v("initial_angle", 3), imu_hz=c.at("imu_hz");
        if (!(imu_hz >= 10 && imu_hz <= 500)) throw std::runtime_error("invalid IMU frequency");
        const bool prime=v("prime_pid", 0), folded=v("fold_angle", 0);
        // Physical constants are required arguments supplied by the YAML loader.
        Plant plant{c.at("body_mass"), c.at("axle_mass"), c.at("com_x"), c.at("com_height"),
                    c.at("pitch_inertia"), c.at("gravity"), c.at("wheel_radius"),
                    c.at("drag"), c.at("angular_damping")};
        const double strength=c.at("max_force"), tau=c.at("motor_lag_ms")/1000;
        const double deadband=c.at("deadband"), polarity=c.at("polarity");
        const double dropout=v("dropout_at", 3), dropout_duration=v("dropout_ms", 0)/1000;
        const double push_at=v("push_at", -1), push=v("push", 15)*rad;
        const int corrupt_every=int(v("corrupt_every", 0));
        // Sensor-to-body rotation Rz(yaw)*Ry(pitch)*Rx(roll). Only x/z
        // projections of body-frame gravity are needed for planar pitch.
        const double roll=c.at("imu_roll_deg")*rad, pitch=c.at("imu_pitch_deg")*rad;
        const double yaw=c.at("imu_yaw_deg")*rad;
        const double r00=std::cos(yaw)*std::cos(pitch);
        const double r02=std::cos(yaw)*std::sin(pitch)*std::cos(roll)+std::sin(yaw)*std::sin(roll);
        const double r20=-std::sin(pitch), r22=std::cos(pitch)*std::cos(roll);
        const double upright=std::atan2(-r20, r22)/rad;
        auto continuous_pitch = [&](double theta) {
            double gx=-r00*std::sin(theta)+r20*std::cos(theta);
            double gz=-r02*std::sin(theta)+r22*std::cos(theta);
            double angle=std::atan2(-gx, gz)/rad;
            return upright + std::remainder(angle-upright, 360.0);
        };
        auto imu_angle = [&](double theta) {
            double angle = continuous_pitch(theta);
            return folded ? std::asin(std::sin(angle*rad))/rad : angle;
        };
        // Runtime overrides are confined to this harness. The .ino is untouched.
        Kp=v("kp", Kp); Ki=v("ki", Ki); Kd=v("kd", Kd);
        setpoint=v("setpoint", originalSetpoint);
        min_mtr_speed=v("min_pwm", min_mtr_speed);
        pid_smpl_time_ms=uint16_t(v("sample_ms", pid_smpl_time_ms));
        if (prime) input=imu_angle(initial*rad);
        setup();
        pid.SetTunings(Kp, Ki, Kd);
        const uint64_t epoch=sim::clock_us;
        sim::pid_updates=sim::imu_reads=sim::fifo_resets=0;
        State state{initial*rad, 0, 0, 0};
        std::vector<double> history;
        std::mt19937 rng(42);
        std::normal_distribution<double> normal(0, 1);
        double force=0, next_imu=1000000/imu_hz, sum_sq=0, max_angle=0;
        unsigned events=0, rows=0, saturation=0, dropped=0, bad=0;
        double fall_time=-1, min_interval=1e9, max_interval=0;
        const int delay_steps=int(std::round(c.at("imu_delay_ms")/1000/dt));
        const int steps=int(std::round(duration/dt));
        std::cout << std::setprecision(10);
        double target=std::atan2(r20*std::cos(setpoint*rad)+r22*std::sin(setpoint*rad),
                                 r00*std::cos(setpoint*rad)+r02*std::sin(setpoint*rad));
        if (std::abs(std::remainder(continuous_pitch(target)-setpoint, 360.0))>90)
            target=std::remainder(target+M_PI, 2*M_PI);
        std::cout << "{\"imu_upright_pitch\":" << imu_angle(0) << ",\"target_body_angle\":" << target/rad
                  << ",\"columns\":[\"t\",\"angle\",\"pitch\",\"omega\",\"x\",\"velocity\",\"output\",\"left\",\"right\",\"force\",\"pid_dt\",\"imu_age\",\"pid_updates\",\"imu_reads\",\"fifo_resets\"],\"samples\":[";
        double last_read_time=-1;
        bool pushed=false;
        for (int step=0; step<=steps; ++step) {
            double t=step*dt;
            if (!pushed && push_at >= 0 && t+1e-9 >= push_at) {
                state[1] += push;
                pushed=true;
            }
            history.push_back(state[0]);
            sim::clock_us=epoch+uint64_t(step)*500;
            if (step*500.0 + 0.01 >= next_imu) {
                next_imu += 1000000/imu_hz;
                ++events;
                if (dropout_duration > 0 && t >= dropout && t < dropout+dropout_duration) ++dropped;
                else {
                    int index=std::max(0, step-delay_steps);
                    double angle=imu_angle(history[index])+normal(rng)*c.at("noise_deg");
                    bool corrupt=corrupt_every>0 && events%corrupt_every==0;
                    if (corrupt) ++bad;
                    mpu.inject(float(angle), corrupt);
                }
            }
            unsigned previous_reads=sim::imu_reads, previous_pid=sim::pid_updates;
            loop();
            if (sim::imu_reads != previous_reads) last_read_time=t;
            if (sim::pid_updates != previous_pid && sim::pid_dt_ms > 0) {
                min_interval=std::min(min_interval, sim::pid_dt_ms);
                max_interval=std::max(max_interval, sim::pid_dt_ms);
            }
            double left=left_motor->command(), right=right_motor->command();
            auto effort = [&](double command) {
                double magnitude=std::max(0.0, (std::abs(command)-deadband)/(255-deadband));
                return std::copysign(magnitude, command);
            };
            double command=polarity*(effort(left)+effort(right))/2;
            // Simplified DC motor back-EMF and first-order force response.
            double target_force=strength*(command-state[3]/c.at("free_speed"));
            force += (target_force-force)*(1-std::exp(-dt/tau));
            if (step%10==0 || step==steps || std::abs(state[0])>=60*rad) {
                if (rows) std::cout << ',';
                double degrees=state[0]/rad;
                double age=last_read_time < 0 ? -1 : (t-last_read_time)*1000;
                std::cout << '[' << t << ',' << degrees << ',' << Pitch << ',' << state[1]/rad
                          << ',' << state[2] << ',' << state[3] << ',' << output << ',' << left
                          << ',' << right << ',' << force << ',' << sim::pid_dt_ms << ',' << age
                          << ',' << sim::pid_updates << ',' << sim::imu_reads << ',' << sim::fifo_resets << ']';
                ++rows; sum_sq+=degrees*degrees;
                max_angle=std::max(max_angle, std::abs(degrees));
                if (std::abs(output)>=254.99) ++saturation;
            }
            if (std::abs(state[0])>=60*rad) { fall_time=t; break; }
            if (step<steps) plant.step(state, force, dt);
        }
        std::cout << "],\"summary\":{\"fall_time\":" << (fall_time<0 ? "null" : std::to_string(fall_time))
                  << ",\"rms_angle\":" << std::sqrt(sum_sq/rows) << ",\"max_angle\":" << max_angle
                  << ",\"saturation_pct\":" << 100.0*saturation/rows
                  << ",\"pid_updates\":" << sim::pid_updates << ",\"imu_reads\":" << sim::imu_reads
                  << ",\"dropped_samples\":" << dropped << ",\"corrupt_samples\":" << bad
                  << ",\"pid_dt_min\":" << (min_interval==1e9 ? 0 : min_interval)
                  << ",\"pid_dt_max\":" << max_interval << "}}\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
