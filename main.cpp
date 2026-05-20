#include <SFML/Graphics.hpp>
#include <cmath>

// ─── PHYSICS CONSTANTS ────────────────────────────────────────────────
const double g = 9.81;
const double M = 1.5;    // cart mass (kg)
const double m = 0.1;    // pole tip mass (kg)
const double l = 1.0;    // pole length (m)

struct State {
    double x;            // cart position
    double x_dot;        // cart velocity
    double theta;        // pole angle (0 = upright)
    double theta_dot;    // pole angular velocity
};

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
// These gains were calculated by solving the Riccati equation for our
// exact system. This is what real control engineers do at companies
// like Epiroc — no guessing, pure math.
//
// F = -(k1*x + k2*x_dot + k3*theta + k4*theta_dot)
//
// Each gain answers one question:
//   k1: cart is away from center → how hard to push back?
//   k2: cart is moving fast      → how hard to slow it?
//   k3: pole is leaning          → how hard to correct?
//   k4: pole is falling fast     → how hard to catch it?
double lqr_controller(State s) {
    // Mathematically calculated gains — not guessed!
    const double k_x     = -22.3607;
    const double k_xdot  = -34.0726;
    const double k_theta = -233.8775;
    const double k_tdot  = -76.2254;

    // F = -K * state  (standard LQR formula)
    double F = -(k_x     * s.x
               + k_xdot  * s.x_dot
               + k_theta * s.theta
               + k_tdot  * s.theta_dot);

    if (F >  500.0) F =  500.0;
    if (F < -500.0) F = -500.0;
    return F;
}

int main() {
    sf::RenderWindow window(sf::VideoMode({1400, 600}),
                            "Inverted Pendulum — LQR Controller");
    window.setFramerateLimit(60);

    // Small initial tilt — 0.1 radians ~ 6 degrees
    State s = {0.0, 0.0, 0.1, 0.0};

    // Fixed timestep — important for stable physics
    // We run physics at 500Hz, graphics at 60Hz
    double dt      = 0.002;
    int    steps   = 8;   // physics steps per frame

    float scale   = 150.0f;
    float groundY = 450.0f;
    float centerX = 700.0f;

    // ── Shapes ────────────────────────────────────────────────────────
    sf::RectangleShape cart({90.f, 40.f});
    cart.setFillColor(sf::Color(70, 100, 210));
    cart.setOrigin({45.f, 20.f});

    sf::RectangleShape pole({8.f, scale * 2.f});
    pole.setFillColor(sf::Color(210, 50, 50));
    pole.setOrigin({4.f, scale * 2.f});

    sf::CircleShape wheel1(11.f), wheel2(11.f);
    wheel1.setFillColor(sf::Color(30, 30, 30));
    wheel2.setFillColor(sf::Color(30, 30, 30));
    wheel1.setOrigin({11.f, 11.f});
    wheel2.setOrigin({11.f, 11.f});

    sf::RectangleShape ground({1400.f, 5.f});
    ground.setFillColor(sf::Color(50, 50, 50));
    ground.setPosition({0.f, groundY + 21.f});

    // Green dot = target position (center)
    sf::CircleShape target(7.f);
    target.setFillColor(sf::Color(0, 200, 0));
    target.setOrigin({7.f, 7.f});
    target.setPosition({700.f, groundY + 22.f});

    // ── Info text shapes (angle indicator bar) ────────────────────────
    // A small bar showing pole angle visually at top of screen
    sf::RectangleShape angleBar({200.f, 14.f});
    angleBar.setFillColor(sf::Color(200, 200, 200));
    angleBar.setPosition({650.f, 20.f});

    sf::RectangleShape angleFill({10.f, 14.f});
    angleFill.setFillColor(sf::Color(250, 50, 50));
    angleFill.setPosition({750.f, 20.f});

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            // Press R to reset simulation
            if (auto* key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::R) {
                    s = {0.0, 0.0, 0.1, 0.0};
                }
            }
        }

        // ── Physics update ────────────────────────────────────────────
        for (int i = 0; i < steps; i++) {
            double F = lqr_controller(s);
            s = rk4_step(s, F, dt);
        }

        // ── Screen coordinates ────────────────────────────────────────
        float cartScreenX = centerX + (float)s.x * scale;

        cart.setPosition({cartScreenX, groundY});
        pole.setPosition({cartScreenX, groundY});
        pole.setRotation(sf::degrees((float)(s.theta * 180.0 / 3.14159)));
        wheel1.setPosition({cartScreenX - 28.f, groundY + 21.f});
        wheel2.setPosition({cartScreenX + 28.f, groundY + 21.f});

        // Angle indicator bar at top — shows how much pole is leaning
        float barFill = (float)(s.theta / 0.5f) * 100.f;
        if (barFill >  100.f) barFill =  100.f;
        if (barFill < -100.f) barFill = -100.f;
        angleFill.setSize({barFill, 14.f});
        angleFill.setPosition({450.f, 20.f});

        // ── Draw ──────────────────────────────────────────────────────
        window.clear(sf::Color(230, 230, 230));
        window.draw(ground);
        window.draw(target);
        window.draw(angleBar);
        window.draw(angleFill);
        window.draw(cart);
        window.draw(wheel1);
        window.draw(wheel2);
        window.draw(pole);
        window.display();
    }

    return 0;
}