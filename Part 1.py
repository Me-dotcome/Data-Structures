import math
from collections import defaultdict
from typing import Dict, List, Optional


class Node:
    """Represents a location/node in the graph"""
    def __init__(self, node_id: str, x: float, y: float):
        self.id = node_id
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f"Node({self.id})"
    
    def __eq__(self, other):
        return self.id == other.id if isinstance(other, Node) else False
    
    def __hash__(self):
        return hash(self.id)


class Edge:
    """Represents a directed edge between two nodes"""
    def __init__(self, source: Node, destination: Node, travel_time: float, traffic_delay: float = 0):
        self.source = source
        self.destination = destination
        self.travel_time = travel_time
        self.traffic_delay = traffic_delay
    
    def get_total_time(self) -> float:
        """Returns total time including traffic"""
        return self.travel_time + self.traffic_delay


class Stop:
    """Represents a stop with deadline and travel time information"""
    def __init__(self, stop_id: str, deadline_time: float, travel_time: float):
        self.stop_id = stop_id
        self.deadline_time = deadline_time
        self.travel_time = travel_time
    
    def __repr__(self):
        return f"Stop({self.stop_id}, deadline={self.deadline_time}, travel_time={self.travel_time})"
    
    def __lt__(self, other):
        """Less than operator for sorting"""
        if self.deadline_time != other.deadline_time:
            return self.deadline_time < other.deadline_time
        return self.travel_time < other.travel_time


class MapGraph:
    """Represents a graph of locations with edges and traffic information"""
    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.edges: Dict[str, List[Edge]] = defaultdict(list)
    
    def add_node(self, node_id: str, x: float = 0, y: float = 0) -> Node:
        """Add a node to the graph"""
        if node_id not in self.nodes:
            self.nodes[node_id] = Node(node_id, x, y)
        return self.nodes[node_id]
    
    def add_edge(self, source_id: str, destination_id: str, travel_time: float, traffic_delay: float = 0):
        """Add a directed edge to the graph"""
        source = self.nodes.get(source_id)
        destination = self.nodes.get(destination_id)
        
        if source and destination:
            edge = Edge(source, destination, travel_time, traffic_delay)
            self.edges[source_id].append(edge)
    
    def getNeighbors(self, node_id: str) -> List[Node]:
        """Get all neighboring nodes from a given node"""
        return [edge.destination for edge in self.edges.get(node_id, [])]
    
    def getTravelTime(self, source_id: str, destination_id: str) -> Optional[float]:
        """Get travel time between two directly connected nodes"""
        for edge in self.edges.get(source_id, []):
            if edge.destination.id == destination_id:
                return edge.travel_time
        return None
    
    def getTrafficDelay(self, source_id: str, destination_id: str) -> Optional[float]:
        """Get traffic delay between two directly connected nodes"""
        for edge in self.edges.get(source_id, []):
            if edge.destination.id == destination_id:
                return edge.traffic_delay
        return None


def CalculateTravelTimeMatrix(all_locations, map_graph):
    """
    Calculate travel times between all pairs of locations using A* algorithm.
    
    Args:
        all_locations: List of all location node IDs
        map_graph: MapGraph object with nodes and edges
    
    Returns:
        matrix: 2D dictionary with travel times from each location to every other location
    """
    # Initialize empty matrix for storing travel times
    matrix = {}
    
    # Iterate through all starting locations
    for start_location in all_locations:
        # Initialize row for this starting location
        matrix[start_location] = {}
        
        # Iterate through all destination locations
        for end_location in all_locations:
            
            if start_location == end_location:
                # Travel time from a location to itself is always 0
                matrix[start_location][end_location] = 0
            else:
                # Run A* algorithm to find the optimal travel time
                time_cost = RunAStar(start_location, end_location, map_graph)
                matrix[start_location][end_location] = time_cost
    
    return matrix


def RunAStar(start, goal, graph):
    """
    A* pathfinding algorithm to find optimal travel time between two locations.
    
    Args:
        start: Starting location node ID
        goal: Goal/destination location node ID
        graph: MapGraph object with methods for getting neighbors, travel times, and traffic delays
    
    Returns:
        float: Optimal travel time from start to goal
    """
    start_node = graph.nodes.get(start)
    goal_node = graph.nodes.get(goal)
    
    if not start_node or not goal_node:
        return float('inf')
    
    # Initialize open list with starting node
    open_list = [start]
    closed_list = set()
    
    # Initialize score dictionaries with infinity for all nodes
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = HeuristicEstimate(start_node, goal_node)
    
    # Main A* loop
    while open_list:
        
        # Find node in open_list with lowest f_score
        current = min(open_list, key=lambda node: f_score[node])
        
        # Check if we reached the goal
        if current == goal:
            return g_score[current]  # Return optimal travel time
        
        # Remove current from open list
        open_list.remove(current)
        # Add current to closed list
        closed_list.add(current)
        
        # Explore all neighbors of current node
        for neighbor_node in graph.getNeighbors(current):
            neighbor = neighbor_node.id
            
            # Skip if neighbor already evaluated
            if neighbor in closed_list:
                continue
            
            # Calculate travel time including traffic conditions
            traffic_delay = graph.getTrafficDelay(current, neighbor) or 0
            distance_time = graph.getTravelTime(current, neighbor) or 0
            tentative_g_score = g_score[current] + distance_time + traffic_delay
            
            # Check if this path is better than previously found path
            if neighbor not in open_list or tentative_g_score < g_score[neighbor]:
                # Update scores for this neighbor
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + HeuristicEstimate(neighbor_node, goal_node)
                
                # Add neighbor to open list if not already there
                if neighbor not in open_list:
                    open_list.append(neighbor)
    
    # No path found from start to goal
    return float('inf')


def HeuristicEstimate(node, goal):
    """
    Calculates estimated travel time from node to goal using Euclidean distance.
    
    Args:
        node: Current location Node object
        goal: Goal location Node object
    
    Returns:
        float: Estimated distance from node to goal
    """
    # Returns estimated distance from node to goal
    # Uses Euclidean distance as heuristic
    dx = goal.x - node.x
    dy = goal.y - node.y
    return math.sqrt(dx**2 + dy**2)


def sort_unvisited_stops(unvisited_stops: List[Stop]) -> List[Stop]:
    """
    Sort unvisited stops by deadline_time ascending, then by travel_time ascending.
    
    Args:
        unvisited_stops: List of Stop objects to sort
    
    Returns:
        Sorted list of stops (sorted by deadline_time, then travel_time)
    """
    return sorted(unvisited_stops, key=lambda stop: (stop.deadline_time, stop.travel_time))


# Example usage
if __name__ == "__main__":
    # Create a sample graph
    graph = MapGraph()
    
    # Add nodes with coordinates
    graph.add_node("A", 0, 0)
    graph.add_node("B", 1, 1)
    graph.add_node("C", 2, 0)
    graph.add_node("D", 1, 2)
    
    # Add edges with travel time and traffic delay
    graph.add_edge("A", "B", travel_time=10, traffic_delay=2)
    graph.add_edge("A", "C", travel_time=15, traffic_delay=3)
    graph.add_edge("B", "D", travel_time=8, traffic_delay=1)
    graph.add_edge("C", "D", travel_time=12, traffic_delay=2)
    graph.add_edge("B", "A", travel_time=10, traffic_delay=2)
    graph.add_edge("D", "A", travel_time=20, traffic_delay=5)
    
    # Calculate the travel time matrix
    locations = ["A", "B", "C", "D"]
    travel_matrix = CalculateTravelTimeMatrix(locations, graph)
    
    # Display the matrix
    print("Travel Time Matrix (in minutes):")
    print("From\\To", end="")
    for loc in locations:
        print(f"\t{loc}", end="")
    print()
    
    for start in locations:
        print(f"{start}", end="")
        for end in locations:
            time = travel_matrix[start][end]
            if time == float('inf'):
                print(f"\t∞", end="")
            else:
                print(f"\t{time:.1f}", end="")
        print()
    
    # Example: Sort unvisited stops by deadline_time, then travel_time
    print("\n" + "="*60)
    print("Sorting Example: Unvisited Stops")
    print("="*60)
    
    # Create some sample stops with deadlines and travel times
    unvisited_stops = [
        Stop("Stop1", deadline_time=15.0, travel_time=5.0),
        Stop("Stop2", deadline_time=10.0, travel_time=3.0),
        Stop("Stop3", deadline_time=10.0, travel_time=2.0),  # Same deadline as Stop2, but shorter travel time
        Stop("Stop4", deadline_time=20.0, travel_time=8.0),
        Stop("Stop5", deadline_time=15.0, travel_time=6.0),  # Same deadline as Stop1, but longer travel time
    ]
    
    print("\nOriginal stops:")
    for stop in unvisited_stops:
        print(f"  {stop}")
    
    # Sort the stops
    sorted_stops = sort_unvisited_stops(unvisited_stops)
    
    print("\nSorted stops (by deadline_time ASC, then travel_time ASC):")
    for stop in sorted_stops:
        print(f"  {stop}")
