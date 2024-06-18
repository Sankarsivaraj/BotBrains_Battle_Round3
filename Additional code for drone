def avoid_collision():
    """Keep an eye out for other drones and avoid bumping into them."""
    while True:
        positions = get_drone_positions()  # Find out where the other drones are
        for position in positions:
            if position != get_own_position() and distance_to(position) < 5:  # Avoid getting too close
                print("Another drone is too close. Changing course.")
                adjust_course()
        time.sleep(0.5)

def distance_to(position):
    """Calculate how far another drone is from us."""
    current_position = get_own_position()
    return np.linalg.norm(np.array(current_position) - np.array(position))

def adjust_course():
    """Change the drone's path to dodge collisions."""
    current_location = vehicle.location.global_frame
    new_location = LocationGlobalRelative(current_location.lat + 0.0001, current_location.lon, current_location.alt)
    vehicle.simple_goto(new_location)
    print(f"Course adjusted to avoid collision. Heading to: {new_location}")
