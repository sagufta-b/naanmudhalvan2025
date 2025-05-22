import time
import math
import logging

# 1. Basic Object Detection (Simulated)
class ObjectDetector:
    def detect(self, image_placeholder):
        print("[Object Detection] Detected: car, pedestrian, stop sign (simulated)")
        return ['car', 'pedestrian', 'stop sign']

# 2. A* Path Planning (Grid)
def astar(grid, start, goal):
    open_set = [start]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        if current == goal:
            return reconstruct_path(came_from, current)

        open_set.remove(current)

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if (0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0])) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.append(neighbor)

    return None

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# 3. Simple Sensor Fusion (Position Averaging)
class SimpleSensorFusion:
    def __init__(self):
        self.readings = []

    def update(self, reading):
        self.readings.append(reading)
        if len(self.readings) > 5:
            self.readings.pop(0)

    def get_estimated_position(self):
        if not self.readings:
            return 0
        return sum(self.readings) / len(self.readings)

# 4. Basic PID Controller
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

# 5. Failsafe System
def emergency_stop(condition):
    if condition:
        print("[FAILSAFE] Emergency stop triggered!")
        return True
    return False

# 6. Performance Metrics Logger (Simple Print)
def log_metrics(task_name, latency, success_rate):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    message = f"[{timestamp}] Task: {task_name}, Latency: {latency:.2f}s, Success Rate: {success_rate*100:.1f}%"
    print("[LOG]", message)
    with open("system_metrics.txt", "a") as f:
        f.write(message + "\n")

# Main Simulation
if __name__ == "__main__":
    print("=== Basic Autonomous System (No External Libraries) ===")

    # 1. Simulated Object Detection
    detector = ObjectDetector()
    objects = detector.detect("placeholder_image")