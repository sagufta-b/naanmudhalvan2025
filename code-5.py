import random
import time
import heapq
import hashlib

# ---------------------- SENSOR SIMULATION ----------------------
class Sensor:
    def read(self):
        raise NotImplementedError

class LiDARSensor(Sensor):
    def read(self):
        return random.uniform(0.5, 10.0)  # distance in meters

class UltrasonicSensor(Sensor):
    def read(self):
        return random.uniform(0.2, 5.0)

class CameraSensor(Sensor):
    def read(self):
        return random.choice(['Clear', 'Obstacle'])

# ---------------------- OBSTACLE DETECTION ----------------------
def detect_obstacle(lidar_val, ultrasonic_val, camera_val):
    return lidar_val < 1.0 or ultrasonic_val < 0.5 or camera_val == 'Obstacle'

# ---------------------- SIMPLE PATH PLANNING ----------------------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        _, current = heapq.heappop(oheap)
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < len(grid):
                if 0 <= neighbor[1] < len(grid[0]):
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return []

# ---------------------- AUTONOMOUS NAVIGATION ----------------------
def navigate(path):
    for step in path:
        print(f"Moving to {step}")
        time.sleep(0.5)
    print("Reached Destination Successfully")

# ---------------------- SECURITY PROTOCOL MOCK ----------------------
def secure_communication(data, key='robotics_secure_key'):
    secure_hash = hashlib.sha256((data + key).encode()).hexdigest()
    return f"Secured Message: {secure_hash}"

# ---------------------- MAIN EXECUTION ----------------------
def main():
    print("Initializing Sensors...")
    lidar = LiDARSensor()
    ultrasonic = UltrasonicSensor()
    camera = CameraSensor()

    print("Reading Sensors...")
    lidar_val = lidar.read()
    ultrasonic_val = ultrasonic.read()
    camera_val = camera.read()

    print(f"LiDAR: {lidar_val:.2f}m, Ultrasonic: {ultrasonic_val:.2f}m, Camera: {camera_val}")

    if detect_obstacle(lidar_val, ultrasonic_val, camera_val):
        print("⚠️ Obstacle Detected! Recalculating Path...")
    else:
        print("✅ Path Clear. Proceeding...")

    # Grid map: 0 = free, 1 = obstacle
    grid = [
        [0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (4, 4)

    path = a_star(grid, start, goal)
    if path:
        navigate(path)
    else:
        print("❌ No viable path found!")

    print(secure_communication("Robot ready for next mission"))

if __name__ == "__main__":
    main()