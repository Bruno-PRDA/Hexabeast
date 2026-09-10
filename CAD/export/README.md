# CAD exports

Drop STEP and STL exports here, then run:

    python tools/measure_cad.py

STL gives exact volumes (hence real part masses); STEP keeps the analytic
cylinders, so the tool can find each servo shaft's true axis and measure the
axis-to-axis link lengths that `robot_config.py` needs.

Regenerate these whenever the parts change - they are derived from the
`.SLDPRT` files one directory up, which remain the source of truth.
