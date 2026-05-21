# Inverted Pendulum Simulator

A real-time inverted pendulum simulator written in C++, compiled to 
WebAssembly and deployed as a web application.

🔗 **Live demo:** [pendulum.nouramouri.com](https://pendulum.nouramouri.com)

<img width="960" height="540" alt="image" src="https://github.com/user-attachments/assets/9484156b-f229-4502-a2f5-cbba68aac7e8" />

## What Is An Inverted Pendulum?

An inverted pendulum is a pole balanced upright on a moving cart. 
It is inherently unstable — gravity pulls it down constantly. 
Keeping it balanced requires a control system that reacts in real time 
to every small disturbance. This is the same control problem behind:

- Autonomous drilling rigs (Epiroc, Atlas Copco)
- Self-balancing robots and Segways
- Rocket stabilization systems
- Humanoid robot balance

## How It Works

### Physics Engine
The equations of motion are derived from Lagrangian mechanics and 
solved using **RK4 (Runge-Kutta 4th order) integration** — running 
at 500 steps per second for high accuracy.

### Controller
An **LQR (Linear Quadratic Regulator)** controller keeps the pole 
balanced. The optimal gains were calculated by solving the continuous 
algebraic Riccati equation using Python and SciPy:

```python
from scipy.linalg import solve_continuous_are
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
```

Calculated gains:
| Gain | Value | Controls |
|------|-------|---------|
| k_x | -22.36 | Cart position |
| k_xdot | -34.07 | Cart velocity |
| k_theta | -233.88 | Pole angle |
| k_tdot | -76.23 | Pole angular velocity |

### Web Deployment
The C++ code is compiled to **WebAssembly** using Emscripten, 
allowing it to run at near-native speed in any browser. 
Graphics are rendered using the HTML Canvas API.

## Tech Stack

| Technology | Purpose |
|------------|---------|
| C++ | Physics engine and LQR controller |
| RK4 Integration | Accurate time-stepping of physics |
| LQR Control | Optimal gain calculation |
| Python + SciPy | Riccati equation solver |
| Emscripten | C++ → WebAssembly compiler |
| WebAssembly | Runs C++ in the browser |
| HTML Canvas + JS | Real-time visualization |
| Vercel | Hosting and custom domain |

## Running Locally

```bash
# Compile with Emscripten
emcc pendulum.cpp -o pendulum.js \
  -s EXPORTED_FUNCTIONS="['_main','_reset_sim','_step_physics','_get_x','_get_theta','_get_force']" \
  -s EXPORTED_RUNTIME_METHODS="['ccall']" \
  -O2

# Serve locally
npx serve .
```

## Controls

- **Reset button** or press **R** — resets the simulation
- **Pause button** or press **Space** — pauses/resumes

## Author

Nour Amouri — [nouramouri.com](https://nouramouri.com)
