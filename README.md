SuspensionSim — Quarter-Car Dynamics

A fully interactive quarter-car suspension simulator built as a dynamic engineering dashboard.

This project models vertical vehicle body motion using second-order differential equations and provides real-time visualization of system behavior, resonance characteristics, and damping performance.

Overview

SuspensionSim simulates a classical quarter-car model, representing:

Sprung mass (vehicle body)

Spring stiffness

Damping coefficient

Road excitation input

The governing equation:

𝑚
𝑥
′
′
+
𝑐
(
𝑥
′
−
𝑟
′
)
+
𝑘
(
𝑥
−
𝑟
)
=
0
mx
′′
+c(x
′
−r
′
)+k(x−r)=0

Where:

m = vehicle mass

c = damping coefficient

k = spring stiffness

x = body displacement

r = road displacement

The system is solved numerically and visualized in an interactive interface.

Features
Engineering Simulation

Time-domain suspension response

Numerical integration of second-order ODE

Road excitation modeling

Stability and resonance behavior analysis

Dynamic Dashboard UI

Real-time parameter sliders

Clean industrial-style interface

Live metric feedback

Performance classification (Good / Warning / Critical)

Frequency Response Analysis

Excitation frequency sweep

Resonance peak identification

Amplitude vs frequency graph

Natural frequency visualization

Parameters Adjustable in Interface

Vehicle mass (kg)

Spring stiffness (N/m)

Damping coefficient (Ns/m)

Road input characteristics

Live recalculation of:

Natural frequency

Damping ratio

System classification

Response amplitude

Engineering Concepts Demonstrated

Newton’s Second Law

Second-order differential systems

Damping ratio classification:

Underdamped

Critically damped

Overdamped

Resonance phenomena

Frequency response curves

Numerical methods for ODE solving

Control-system style system behavior interpretation

Technical Stack

Frontend:

HTML5

CSS3 (custom industrial UI design)

Responsive layout with grid system

Custom engineering-themed design system

Simulation Logic:

Second-order dynamic system modeling

Numerical integration

Parameter-driven recalculation

Why This Project Matters

This project demonstrates:

Applied physics knowledge (Physics I)

Calculus II concepts (second-order differential equations)

Numerical modeling skills

Engineering system interpretation

Clean UI/UX thinking for technical tools

Self-directed project development

It bridges mechanical engineering fundamentals with software implementation.

Future Improvements

Planned extensions:

Unsprung mass (full quarter-car model)

Tire stiffness modeling

Random road profile input

Comfort metric (RMS acceleration)

Optimization module for ride comfort vs performance

Streamlit / Web deployment version

GitHub Pages deployment

How to Run

If simulation is Python-based:

pip install -r requirements.txt
python main.py

If using web version:

Open the HTML file in a browser.

Author
Obi Umeh 
Aspiring Mechanical Engineer
Interested in dynamic systems, simulation, and applied physics
