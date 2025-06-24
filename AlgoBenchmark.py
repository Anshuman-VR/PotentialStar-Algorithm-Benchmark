import pygame
import numpy as np
import math
import random
import time
import pickle
from queue import PriorityQueue
from collections import deque


GRID_SIZE = 30
CELL_SIZE = 20
WIDTH, HEIGHT = GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE
FAST_MODE = False
ALGORITHMS = ["A*", "Potential Field + A*", "BFS"]


COLORS = {
    "empty": (255, 255, 255),
    "start": (255, 0, 0),
    "goal": (0, 255, 0),
    "wall": (0, 0, 0),
    "visited": (0, 255, 255),
    "path": (160, 32, 240),
    "fallback_path": (255, 165, 0),
    "grid": (200, 200, 200),
}


pygame.init()
window = pygame.display.set_mode((WIDTH, HEIGHT + 150))
pygame.display.set_caption("Benchmark Test Environment")
font = pygame.font.SysFont("Arial", 20)

grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
start = None
goal = None
current_algo_idx = 0
stats_text = ""


def draw_grid():
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell_value = grid[y, x]
            color = COLORS["empty"]
            if cell_value == 1:
                color = COLORS["start"]
            elif cell_value == 2:
                color = COLORS["goal"]
            elif cell_value == 3:
                color = COLORS["wall"]
            elif cell_value == 4:
                color = COLORS["visited"]
            elif cell_value == 5:
                color = COLORS["path"]
            elif cell_value == 6:
                color = COLORS["fallback_path"]
            pygame.draw.rect(window, color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(window, COLORS["grid"], (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_neighbors(pos):
    y, x = pos
    for dy, dx in [(-1,0), (1,0), (0,1), (0,-1)]:
        ny, nx = y + dy, x + dx
        if 0 <= ny < GRID_SIZE and 0 <= nx < GRID_SIZE:
            if grid[ny, nx] != 3:
                yield (ny, nx)

def reconstruct_path(came_from, end, mark=5):
    path_length = 0
    current = end
    while current in came_from:
        current = came_from[current]
        if grid[current] not in (1, 2):
            grid[current] = mark
            path_length += 1
        if not FAST_MODE:
            draw_grid()
            pygame.display.flip()
    return path_length

def generate_obstacles(density=0.25):
    global grid, start, goal
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    start, goal = None, None
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if random.random() < density:
                grid[y, x] = 3

def save_map(filename="map.pkl"):
    with open(filename, "wb") as f:
        pickle.dump((grid, start, goal), f)

def load_map(filename="map.pkl"):
    global grid, start, goal
    with open(filename, "rb") as f:
        grid, start, goal = pickle.load(f)



def run_astar(start, goal):
    t0 = time.time()
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while not open_set.empty():
        _, current = open_set.get()
        if current == goal:
            path_len = reconstruct_path(came_from, current)
            t1 = time.time()
            return len(visited), path_len, t1 - t0
        visited.add(current)

        for neighbor in get_neighbors(current):
            temp_g = g_score[current] + 1
            if neighbor not in g_score or temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score = temp_g + manhattan(neighbor, goal)
                open_set.put((f_score, neighbor))
                if grid[neighbor] not in (1, 2):
                    grid[neighbor] = 4

        if not FAST_MODE:
            draw_grid()
            pygame.display.flip()

    t1 = time.time()
    return len(visited), 0, t1 - t0

def run_bfs(start, goal):
    t0 = time.time()
    queue = deque([start])
    came_from = {}
    visited = set([start])

    while queue:
        current = queue.popleft()
        if current == goal:
            path_len = reconstruct_path(came_from, current)
            t1 = time.time()
            return len(visited), path_len, t1 - t0

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
                if grid[neighbor] not in (1, 2):
                    grid[neighbor] = 4

        if not FAST_MODE:
            draw_grid()
            pygame.display.flip()

    t1 = time.time()
    return len(visited), 0, t1 - t0


def run_astar_escape(start, goal, node_limit=None):
    t0 = time.time()
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()
    nodes_explored = 0

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            path_len = reconstruct_path(came_from, current)
            t1 = time.time()
            print(f"[A*] Goal reached! Nodes explored: {len(visited)}")
            return len(visited), path_len, t1 - t0, came_from

        if current in visited:
            continue
        visited.add(current)
        nodes_explored += 1

        if node_limit is not None and nodes_explored >= node_limit:
            t1 = time.time()
            print(f"[A*] Node limit reached: {node_limit}. Nodes explored: {len(visited)}")
            return len(visited), 0, t1 - t0, came_from

        for neighbor in get_neighbors(current):
            temp_g = g_score[current] + 1
            if neighbor not in g_score or temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score = temp_g + manhattan(neighbor, goal)
                open_set.put((f_score, neighbor))
                if grid[neighbor] not in (1, 2, 3):
                    grid[neighbor] = 4

        if not FAST_MODE:
            draw_grid()
            pygame.display.flip()

    t1 = time.time()
    print(f"[A*] No path found. Nodes explored: {len(visited)}")
    return len(visited), 0, t1 - t0, came_from

def run_potential_field_with_astar(start, goal):
    t0 = time.time()
    current = start
    came_from_pf = {}
    visited = set()
    path_length = 0
    prev_positions = deque(maxlen=10)

    def compute_force(pos):
        dy, dx = goal[0] - pos[0], goal[1] - pos[1]
        dist = math.hypot(dy, dx) + 1e-6
        attr_y, attr_x = dy / dist, dx / dist

        rep_y, rep_x = 0.0, 0.0
        for ny, nx in get_neighbors(pos):
            if grid[ny, nx] == 3:
                dy_o, dx_o = pos[0] - ny, pos[1] - nx
                dist_o = math.hypot(dy_o, dx_o) + 1e-6
                if dist_o < 3.0:
                    strength = 5.0 * (1.0 / dist_o - 1.0 / 3.0) / (dist_o ** 2)
                    rep_y += strength * dy_o
                    rep_x += strength * dx_o

        return attr_y + rep_y, attr_x + rep_x

    while current != goal:
        visited.add(current)
        prev_positions.append(current)

        fy, fx = compute_force(current)
        best_neighbor = None
        best_dot =  -float('inf')

        for ny, nx in get_neighbors(current):
            vy, vx = ny - current[0], nx - current[1]
            dot = fy * vy + fx * vx
            if dot > best_dot:
                best_dot = dot
                best_neighbor = (ny, nx)

        if best_neighbor and best_dot > 0:
            came_from_pf[best_neighbor] = current
            current = best_neighbor
            path_length += 1
            if grid[current] not in (1, 2, 3):
                grid[current] = 5
        else:
            print(f"[PF] Got Stuck at {current}. Switching to A*")
            nodes, astar_path_len, _, came_from_astar = run_astar_escape(current, goal, node_limit=500)
            if came_from_astar and goal in came_from_astar:
                reconstruct_path(came_from_pf, current, mark=5)
                reconstruct_path(came_from_astar, goal, mark=5)
                t1 = time.time()
                print(f"[PF+A*] A* Switch successful")
                return len(visited) + nodes, path_length + astar_path_len, t1 - t0
            else:
                print(f"[PF+A*] A* failed to escape. Giving up.")
                t1 = time.time()
                return len(visited) + nodes, path_length, t1 - t0

        # Checking if no progress has been made
        if len(prev_positions) >= 5:
            dists = [euclidean(p, goal) for p in prev_positions]
            if max(dists) - min(dists) < 2:
                print(f"[PF] No progress at {current}. Switching to A*")
                nodes, astar_path_len, _, came_from_astar = run_astar_escape(current, goal, node_limit=500)
                if came_from_astar and goal in came_from_astar:
                    reconstruct_path(came_from_pf, current, mark=5)
                    reconstruct_path(came_from_astar, goal, mark=5)
                    t1 = time.time()
                    print(f"[PF+A*] Fallback A* succeeded")
                    return len(visited) + nodes, path_length + astar_path_len, t1 - t0
                else:
                    print(f"[PF+A*] Fallback A* failed to escape. Giving up.")
                    t1 = time.time()
                    return len(visited) + nodes, path_length, t1 - t0

        if not FAST_MODE:
            draw_grid()
            pygame.display.flip()

    t1 = time.time()
    reconstruct_path(came_from_pf, goal, mark=5)
    print(f"[PF] Goal reached without fallback.")
    return len(visited), path_length, t1 - t0


running = True
while running:
    window.fill(COLORS["empty"])
    draw_grid()
    window.blit(font.render(f"Algorithm: {ALGORITHMS[current_algo_idx]}", True, (0,0,0)), (5, HEIGHT + 5))
    window.blit(font.render(f"Left Click: Place Start, Goal & Obstacles, Right Click: Clear Cell", True, (0,0,0)), (5, HEIGHT + 30))
    window.blit(font.render(f"TAB: Change, SPACE: Run, M:Generate Random Obstacles", True, (0,0,0)), (5, HEIGHT + 55))
    window.blit(font.render(f"R: Clear Grid, S: Save Grid, L: Load Grid", True, (0,0,0)), (5, HEIGHT + 75))
    window.blit(font.render(f"Fast Mode: {'ON' if FAST_MODE else 'OFF'} (F to toggle)", True, (0,0,0)), (5, HEIGHT + 95))
    window.blit(font.render(stats_text, True, (0,0,0)), (5, HEIGHT + 115))
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if pygame.mouse.get_pressed()[0]:
            mx, my = pygame.mouse.get_pos()
            if my < HEIGHT:
                gx, gy = mx // CELL_SIZE, my // CELL_SIZE
                if not start:
                    grid[gy, gx] = 1
                    start = (gy, gx)
                elif not goal and grid[gy, gx] != 1:
                    grid[gy, gx] = 2
                    goal = (gy, gx)
                elif grid[gy, gx] == 0:
                    grid[gy, gx] = 3

        if pygame.mouse.get_pressed()[2]:
            mx, my = pygame.mouse.get_pos()
            if my < HEIGHT:
                gx, gy = mx // CELL_SIZE, my // CELL_SIZE
                if grid[gy, gx] == 1:
                    start = None
                if grid[gy, gx] == 2:
                    goal = None
                grid[gy, gx] = 0

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_TAB:
                current_algo_idx = (current_algo_idx + 1) % len(ALGORITHMS)
            if event.key == pygame.K_SPACE and start and goal:
                grid[grid == 4] = 0
                grid[grid == 5] = 0
                grid[grid == 6] = 0
                if ALGORITHMS[current_algo_idx] == "A*":
                    nodes, path_len, t = run_astar(start, goal)
                elif ALGORITHMS[current_algo_idx] == "BFS":
                    nodes, path_len, t = run_bfs(start, goal)
                else:
                    nodes, path_len, t = run_potential_field_with_astar(start, goal)
                stats_text = f"Explored: {nodes} | Path: {path_len} | Time: {t:.4f}s"
            if event.key == pygame.K_r:
                grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
                start, goal = None, None
                stats_text = ""
            if event.key == pygame.K_m:
                generate_obstacles()
            if event.key == pygame.K_s:
                save_map()
            if event.key == pygame.K_l:
                load_map()
            if event.key == pygame.K_f:
                FAST_MODE = not FAST_MODE

pygame.quit()
