import math

def simulate():
    # Rover physical state
    rover_x = 0.0
    rover_y = 0.0
    rover_heading_deg = 0.0  # 0 = East, 90 = South (if Y is Down), 90 = North (if Y is Up)
    
    # Map coordinate system:
    # Let's assume standard math (Y goes UP).
    # target = A1 = (0, 6.5)
    target_x = 0.0
    target_y = 6.5
    
    # ESP8266 Physical Wiring:
    # if swapped == True, M1 pin drives Left motor, M2 pin drives Right motor.
    swapped_wiring = True
    
    # Python Payload:
    # if swapped_payload == True, sends "right, left"
    swapped_payload = True
    
    for step in range(10):
        # 1. UWB System computes heading
        dx = target_x - rover_x
        dy = target_y - rover_y
        target_angle = math.atan2(dy, dx)
        
        current_heading_rad = math.radians(rover_heading_deg)
        heading_error = target_angle - current_heading_rad
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        
        # 2. PID
        kp = 50.0
        bias = int(heading_error * kp)
        base = 225
        
        left_speed = base - bias
        right_speed = base + bias
        
        left_speed = max(190, min(255, left_speed))
        right_speed = max(190, min(255, right_speed))
        
        # 3. Payload
        if swapped_payload:
            payload = f"{right_speed},{left_speed}"
        else:
            payload = f"{left_speed},{right_speed}"
            
        # 4. ESP parses
        parts = payload.split(",")
        esp_left = int(parts[0])
        esp_right = int(parts[1])
        
        # 5. Chassis acts
        if swapped_wiring:
            physical_left = esp_right
            physical_right = esp_left
        else:
            physical_left = esp_left
            physical_right = esp_right
            
        # 6. Physics updates heading
        # If Physical Right is faster, it turns LEFT (increases angle in standard math)
        turn_rate = (physical_right - physical_left) * 0.5  # Arbitrary scalar
        
        print(f"Step {step} | Err: {math.degrees(heading_error):5.1f} | L_speed={left_speed} R_speed={right_speed} | PhysL={physical_left} PhysR={physical_right} | Turn={turn_rate:5.1f} | Heading: {rover_heading_deg:5.1f} -> {rover_heading_deg + turn_rate:5.1f}")
        
        rover_heading_deg += turn_rate
        
simulate()
