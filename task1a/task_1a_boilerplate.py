#!/usr/bin/env python3
"""
Task 1A: PacBot Path Planning with A*
"""

import json
import heapq
import time

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"         
PELLETS_TOPIC = "pellets/pose"    
CMD_VEL_TOPIC = "robot/cmd_vel"  

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}

DELTA_TO_YAW = {
    (0, 1): 0.0,
    (1, 0): 90.0,
    (0, -1): 180.0,
    (-1, 0): 270.0,
}

EXIT_FACING_YAW = {
    'east': 0.0,
    'north': 90.0,
    'west': 180.0,
    'south': 270.0,
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================

def heuristic(cell_a, cell_b):
    """Manhattan distance heuristic for grid navigation."""
    return abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1])


def get_neighbors(cell):
    """Returns valid reachable neighbor cells based on maze walls."""
    r, c = cell
    neighbors = []
    cell_wall_mask = WALLS[r][c]

    for yaw, (dr, dc, wall_bit) in HEADING_DELTA.items():
        # Check if there is a wall blocking this direction in current cell
        if not (cell_wall_mask & wall_bit):
            nr, nc = r + dr, c + dc
            if 0 <= nr < MAZE_ROWS and 0 <= nc < MAZE_COLS:
                neighbors.append((nr, nc))
    return neighbors


def a_star_search(start_cell, goal_cell):
    """
    Computes the shortest coordinate path from start_cell to goal_cell using A*.
    Returns a list of coordinates [(r0, c0), (r1, c1), ...] or None if unreachable.
    """
    if start_cell == goal_cell:
        return [start_cell]

    open_set = []
    heapq.heappush(open_set, (0, start_cell))
    
    came_from = {}
    g_score = {start_cell: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_cell:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in get_neighbors(current):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal_cell)
                heapq.heappush(open_set, (f_score, neighbor))

    return None


def get_relative_turn_command(current_yaw, target_yaw):
    """
    Converts current orientation and desired absolute heading into
    robot commands: 'FRONT', 'LEFT', 'RIGHT', 'BACK'.
    """
    turn_diff = (target_yaw - current_yaw) % 360.0

    if turn_diff == 0.0:
        return "FRONT"
    elif turn_diff == 90.0:
        return "LEFT"
    elif turn_diff == 180.0:
        return "BACK"
    elif turn_diff == 270.0:
        return "RIGHT"
    return "FRONT"


def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    """
    FRONT/LEFT/RIGHT/BACK to send now, or None. 
    pacbot_cell=(row,col), pacbot_yaw one of HEADING_DELTA's keys,
    pellets_remaining=set of (row,col).
    """
    # 1. Select the next target
    if pellets_remaining:
        # Find the shortest A* path among all remaining pellets
        best_path = None
        for pellet in pellets_remaining:
            path = a_star_search(pacbot_cell, pellet)
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path

        if best_path is None or len(best_path) < 2:
            return None

        # Determine target yaw to enter the next cell
        next_cell = best_path[1]
        dr = next_cell[0] - pacbot_cell[0]
        dc = next_cell[1] - pacbot_cell[1]
        desired_yaw = DELTA_TO_YAW[(dr, dc)]

        return get_relative_turn_command(pacbot_yaw, desired_yaw)

    # 2. All pellets collected: Navigate to the nearest exit
    best_exit_path = None
    target_exit_meta = None

    for exit_row, exit_col, facing in EXIT_CELLS:
        exit_cell = (exit_row, exit_col)
        path = a_star_search(pacbot_cell, exit_cell)
        if path and (best_exit_path is None or len(path) < len(best_exit_path)):
            best_exit_path = path
            target_exit_meta = (exit_cell, facing)

    if best_exit_path is None:
        return None

    # Bot is in transit to the exit cell
    if len(best_exit_path) > 1:
        next_cell = best_exit_path[1]
        dr = next_cell[0] - pacbot_cell[0]
        dc = next_cell[1] - pacbot_cell[1]
        desired_yaw = DELTA_TO_YAW[(dr, dc)]
        return get_relative_turn_command(pacbot_yaw, desired_yaw)

    # Bot has arrived at the exit cell: Turn/Step out through the exit
    exit_cell, facing = target_exit_meta
    desired_yaw = EXIT_FACING_YAW[facing]
    return get_relative_turn_command(pacbot_yaw, desired_yaw)

# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)  
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()