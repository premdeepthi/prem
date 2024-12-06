import rubrikscu
from rubrikscu import *  # Import Rubik's cube module.
import time
from collections import deque  # Deque for efficient BFS queues.
import heapq  # Importing heapq for the priority queue in A* search.


# Shortest path - BFS function
# This function uses BFS to find the shortest sequence of moves to transform the Rubik's cube 
# from `start_state` to `end_state`.

def shortest_path(start_state, end_state):
    """
    Finds the shortest path between start_state and end_state using BFS.

    Args:
        start_state (tuple): The initial state of the cube.
        end_state (tuple): The goal state of the cube.

    Returns:
        list: A sequence of moves to solve the cube, or None if no solution exists.
    """
    visited = {}  # Dictionary to store visited states and their parent state/move.
    visited[start_state] = (start_state, -1)  # Initialize with the start state and no move.
    queue = deque([start_state])  # Initialize the BFS queue with the start state.

    while queue:
        current_state = queue.popleft()  # Get the next state to process.

        if current_state == end_state:  # If the goal state is reached, stop searching.
            break

        # Apply all possible quarter twists (cube moves).
        for twist in rubrikscu.quarter_twists:
            next_state = rubrikscu.perm_apply(current_state, twist)  # Get the next state.

            if next_state not in visited:  # If the next state has not been visited:
                visited[next_state] = (current_state, rubrikscu.quarter_twists_names[twist])  # Track its parent and move.
                queue.append(next_state)  # Add it to the queue for further exploration.

    # If the only visited state is the start state, return an empty path (no moves needed).
    if len(visited) == 1:
        return []

    if end_state in visited:  # If the goal state was reached, reconstruct the solution path.
        path = []
        current_node = end_state

        # Backtrack from the goal state to the start state.
        while current_node != start_state:
            prev_node, current_twist = visited[current_node]  # Get the parent state and move.
            path.append(current_twist)  # Add the move to the path.
            current_node = prev_node  # Move to the parent state.

        return list(reversed(path))  # Reverse the path to get it from start to end.

    else:  # If the goal state was not reached, return None.
        return None

import heapq  # Importing heapq for the priority queue in A* search.

# Heuristic for A* search: Manhattan distance based on misplaced pieces
def manhattan_distance(state):
    # This function calculates the Manhattan distance heuristic for A*.
    # It returns the number of misplaced pieces (or a simple heuristic).
    return sum(1 for i in range(len(state)) if state[i] != I[i])  # I is the solved state.

# A* algorithm implementation
def a_star(start_state, end_state):
    # open_set: Priority queue storing states along with their f_score and g_score.
    open_set = []
    # Start state with f_score = g_score + heuristic (Manhattan distance)
    heapq.heappush(open_set, (0 + manhattan_distance(start_state), 0, start_state))  # (f_score, g_score, state)
    
    # Dictionary to track the state path (came_from stores the previous state and the action taken)
    came_from = {}

    # g_score dictionary to track the cost (number of moves) to reach each state from start
    g_score = {start_state: 0}
    
    # Start the A* search loop
    while open_set:
        # Pop the state with the lowest f_score (best state to explore next)
        _, current_g_score, current_state = heapq.heappop(open_set)
        
        # If we reached the end state, reconstruct the path and return it
        if current_state == end_state:
            path = []
            while current_state in came_from:
                prev_state, action = came_from[current_state]  # Get the previous state and action.
                path.append(action)  # Add the action to the path.
                current_state = prev_state  # Move to the previous state.
            return path[::-1]  # Return the path in the correct order (start -> end).

        # Explore neighbors by applying possible quarter_twists (moves)
        for action in quarter_twists:
            # Get the new state after applying the move (twist)
            neighbor_state = perm_apply(action, current_state)
            tentative_g_score = current_g_score + 1  # Increment move count (g_score)

            # Check if this new state offers a better (lower) g_score
            if neighbor_state not in g_score or tentative_g_score < g_score[neighbor_state]:
                g_score[neighbor_state] = tentative_g_score  # Update g_score for this state
                f_score = tentative_g_score + manhattan_distance(neighbor_state)  # Calculate f_score (g_score + heuristic)
                heapq.heappush(open_set, (f_score, tentative_g_score, neighbor_state))  # Push to open_set with f_score
                came_from[neighbor_state] = (current_state, quarter_twists_names[action])  # Store parent state and move

    return None  # Return None if no path is found (no solution exists)



# Shortest path - Bidirectional BFS
# This function uses bidirectional BFS to find the shortest sequence of moves 
# between start_state and end_state.

def shortest_path_optimized(start_state, end_state):
    """
    Finds the shortest path using bidirectional BFS for improved efficiency.

    Args:
        start_state (tuple): The initial state of the cube.
        end_state (tuple): The goal state of the cube.

    Returns:
        list: A sequence of moves to solve the cube, or None if no solution exists.
    """
    if start_state == end_state:  # If already solved, return an empty path.
        return []

    # Initialize data structures for bidirectional search.
    start_parent = {}  # Parent states for forward search.
    end_parent = {}  # Parent states for backward search.
    visited_start = set()  # Visited states in forward search.
    visited_end = set()  # Visited states in backward search.
    start_queue = deque([start_state])  # Queue for forward search.
    end_queue = deque([end_state])  # Queue for backward search.
    start_dist = {start_state: 0}  # Distance from the start state.
    end_dist = {end_state: 0}  # Distance from the end state.
    found = False  # Flag to indicate if a solution is found.

    while start_queue or end_queue:
        # Forward BFS step.
        if start_queue:
            current_state = start_queue.popleft()

            # Apply all possible moves in the forward direction.
            for action in rubrikscu.quarter_twists:
                neighbor_state = rubrikscu.perm_apply(action, current_state)

                if neighbor_state in start_dist:  # If already visited:
                    if start_dist[neighbor_state] > start_dist[current_state] + 1:
                        start_dist[neighbor_state] = start_dist[current_state] + 1
                        start_parent[neighbor_state] = (current_state, action)
                        start_queue.append(neighbor_state)
                else:  # If not visited:
                    start_dist[neighbor_state] = start_dist[current_state] + 1
                    start_parent[neighbor_state] = (current_state, action)
                    start_queue.append(neighbor_state)

            visited_start.add(current_state)  # Mark as visited.

        # Check if forward search meets backward search.
        if current_state in visited_end and current_state in visited_start:
            found = True
            break

        # Backward BFS step.
        if end_queue:
            current_state = end_queue.popleft()

            # Apply all possible moves in the backward direction.
            for action in rubrikscu.quarter_twists:
                neighbor_state = rubrikscu.perm_apply(action, current_state)

                if neighbor_state in end_dist:  # If already visited:
                    if end_dist[neighbor_state] > end_dist[current_state] + 1:
                        end_dist[neighbor_state] = end_dist[current_state] + 1
                        end_parent[neighbor_state] = (current_state, action)
                        end_queue.append(neighbor_state)
                else:  # If not visited:
                    end_dist[neighbor_state] = end_dist[current_state] + 1
                    end_parent[neighbor_state] = (current_state, action)
                    end_queue.append(neighbor_state)

            visited_end.add(current_state)  # Mark as visited.

        # Check if backward search meets forward search.
        if current_state in visited_start and current_state in visited_end:
            found = True
            break

    if found:
        # Combine paths from forward and backward searches.
        inverse_twists = {F: Fi, L: Li, U: Ui, Fi: F, Li: L, Ui: U}  # Inverse move mappings.
        moves_to_start = [rubrikscu.quarter_twists_names[start_parent[current_state][1]]]
        parent = start_parent[current_state][0]

        # Reconstruct the path from start to meeting point.
        while parent != start_state:
            moves_to_start.append(rubrikscu.quarter_twists_names[start_parent[parent][1]])
            parent = start_parent[parent][0]

        moves_to_end = []

        # Reconstruct the path from meeting point to end.
        if current_state != end_state:
            moves_to_end = [rubrikscu.quarter_twists_names[inverse_twists[end_parent[current_state][1]]]]
            parent = end_parent[current_state][0]

            while parent != end_state:
                moves_to_end.append(rubrikscu.quarter_twists_names[inverse_twists[end_parent[parent][1]]])
                parent = end_parent[parent][0]

        return moves_to_start[::-1] + moves_to_end  # Combine forward and backward paths.

    else:
        return None  # If no solution found, return None.

# Measure the time taken by BFS.
start = time.time()
path = shortest_path((6, 7, 8, 20, 18, 19, 3, 4, 5, 16, 17, 15, 0, 1, 2, 14, 12, 13, 10, 11, 9, 21, 22, 23),
                     (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23))
end = time.time()
print("PATH found by normal bfs:" + str(path))
print("time in secs by normal bfs:" + str(end - start))

# Timer for A* Search
start = time.time()
path = a_star((6, 7, 8, 20, 18, 19, 3, 4, 5, 16, 17, 15, 0, 1, 2, 14, 12, 13, 10, 11, 9, 21, 22, 23),
                (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23))
end = time.time()

print("PATH found by A* search:" + str(path))
print("time in secs by A* search:" + str( end - start))

# Measure the time taken by bidirectional BFS.
start = time.time()
path = shortest_path_optimized((6, 7, 8, 20, 18, 19, 3, 4, 5, 16, 17, 15, 0, 1, 2, 14, 12, 13, 10, 11, 9, 21, 22, 23),
                                (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23))
end = time.time()
print("PATH found by bidirectional bfs:" + str(path))
print("time in secs by bidirectional bfs:" + str(end - start))


