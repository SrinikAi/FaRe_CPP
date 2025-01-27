import math

def calculate_fov_and_radius():
  
    h = 65  
    a = 70  
    b = 130  
    d1 = 15  
    d2 = 115 

    # 1. Calculate Vertical Field of View (FoV)
    delta_y = b - a  # Difference in side lengths
    theta = 2 * math.degrees(math.atan(delta_y / (2 * h)))  # Vertical FoV in degrees

    # 2. Calculate Horizontal Field of View (FoV)
    phi_half = math.degrees(math.atan(b / (2 * d2)))  # Half of horizontal FoV in degrees
    phi = 2 * phi_half  # Total horizontal FoV in degrees

    # 3. Calculate Radius of the trapezoidal area
    radius = math.sqrt(d2**2 + (b / 2)**2)  # Radius in cm

    # Print results
    print(f"Vertical Field of View (FoV): {theta:.2f} degrees")
    print(f"Horizontal Field of View (FoV): {phi:.2f} degrees")
    print(f"Radius of trapezoidal area: {radius:.2f} cm")


calculate_fov_and_radius()
