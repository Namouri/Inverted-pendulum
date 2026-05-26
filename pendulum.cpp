#include <emscripten.h>
#include <emscripten/html5.h>
#include <cmath>
#include <cstring>
#include <cstdio>


// ─── PHYSICS CONSTANTS ────────────────────────────────────────────────
// Exactly the same as our C++ version — nothing changed here
const double g = 9.81;
const double M = 1.5;
const double m = 0.1;
const double l = 1.0;

// ─── STATE ────────────────────────────────────────────────────────────
// Same 4 numbers describing the system at any moment
struct State {
    double x;
    double x_dot;
    double theta;
    double theta_dot;
};

// ─── PHYSICS ──────────────────────────────────────────────────────────
// Identical to our C++ version
State derivatives(State s, double F) {
    double sin_t = sin(s.theta);
    double cos_t = cos(s.theta);
    double denom = l * (4.0/3.0 - m * cos_t * cos_t / (M + m));
    double theta_ddot = (g * sin_t - cos_t *
                        (F + m * l * s.theta_dot * s.theta_dot * sin_t)
                        / (M + m)) / denom;
    double x_ddot = (F + m * l * (s.theta_dot * s.theta_dot * sin_t
                   - theta_ddot * cos_t)) / (M + m);
    return {s.x_dot, x_ddot, s.theta_dot, theta_ddot};
}

// ─── RK4 ──────────────────────────────────────────────────────────────
// Identical to our C++ version
State rk4_step(State s, double F, double dt) {
    State k1 = derivatives(s, F);
    State s2 = {s.x + k1.x*dt/2, s.x_dot + k1.x_dot*dt/2,
                s.theta + k1.theta*dt/2, s.theta_dot + k1.theta_dot*dt/2};
    State k2 = derivatives(s2, F);
    State s3 = {s.x + k2.x*dt/2, s.x_dot + k2.x_dot*dt/2,
                s.theta + k2.theta*dt/2, s.theta_dot + k2.theta_dot*dt/2};
    State k3 = derivatives(s3, F);
    State s4 = {s.x + k3.x*dt, s.x_dot + k3.x_dot*dt,
                s.theta + k3.theta*dt, s.theta_dot + k3.theta_dot*dt};
    State k4 = derivatives(s4, F);
    return {
        s.x         + dt/6*(k1.x         + 2*k2.x         + 2*k3.x         + k4.x),
        s.x_dot     + dt/6*(k1.x_dot     + 2*k2.x_dot     + 2*k3.x_dot     + k4.x_dot),
        s.theta     + dt/6*(k1.theta     + 2*k2.theta     + 2*k3.theta     + k4.theta),
        s.theta_dot + dt/6*(k1.theta_dot + 2*k2.theta_dot + 2*k3.theta_dot + k4.theta_dot)
    };
}

// ─── LQR CONTROLLER ───────────────────────────────────────────────────
// Identical gains calculated by Python/Riccati equation
double lqr_controller(State s) {
    const double k_x     = -22.3607;
    const double k_xdot  = -34.0726;
    const double k_theta = -233.8775;
    const double k_tdot  = -76.2254;

    double F = -(k_x     * s.x
               + k_xdot  * s.x_dot
               + k_theta * s.theta
               + k_tdot  * s.theta_dot);

    if (F >  500.0) F =  500.0;
    if (F < -500.0) F = -500.0;
    return F;
}

// ─── SIMULATION STATE ─────────────────────────────────────────────────
// Global so it persists between frames
State s = {0.0, 0.0, 0.1, 0.0};
double dt = 0.002;
double disturbance_force = 0.0;  // extra force applied by user
int disturbance_frames = 0;


extern "C" {
    // These functions are called FROM JavaScript
    void reset_sim() {
        s = {0.0, 0.0, 0.0, 0.0};
        disturbance_force = 0.0;
        disturbance_frames = 0;


    }

    // JavaScript calls this every frame to get the current state
    double get_x()         { return s.x; }
    double get_theta()     { return s.theta; }
    double get_force()     { return lqr_controller(s); }
    void apply_disturbance(double force) { 
        disturbance_force = force;
        disturbance_frames = 30;  // apply for 30 frames — visible effect

     }


    // JavaScript calls this to advance the physics
    void step_physics() {
        double extra = 0.0;
       if (disturbance_frames > 0) {
        extra = disturbance_force;
        disturbance_frames--;
    }
    for (int i = 0; i < 8; i++) {
        double F = lqr_controller(s) + extra;
        s = rk4_step(s, F, dt);
    };
    }

   

}

int main() { return 0; }