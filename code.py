import math
from collections import defaultdict

def haversine(coord1, coord2):
    R = 6371.0
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

class Node:
    def __init__(self, name, location):
        self.name = name
        self.location = location
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')
        self.parent = None

class TransportRoute:
    def __init__(self, name, mode, class_type, base_cost, base_time, average_delay, transfer_time, stop_time, expected_waiting):
        self.name = name
        self.mode = mode
        self.class_type = class_type
        self.base_cost = base_cost
        self.base_time = base_time
        self.average_delay = average_delay
        self.transfer_time = transfer_time
        self.stop_time = stop_time
        self.expected_waiting = expected_waiting

    def calculate_total_cost(self):
        return self.base_cost  

    def calculate_total_time(self):
        return (self.base_time + self.stop_time + self.average_delay + self.expected_waiting + self.transfer_time)

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = defaultdict(dict)

    def add_node(self, name, location):
        self.nodes[name] = Node(name, location)

    def add_edge(self, from_node, to_node, route):
        self.edges[from_node][to_node] = route

    def get_neighbors(self, node):
        return self.edges[node.name].keys()

    def get_route(self, from_node, to_node):
        return self.edges[from_node.name][to_node.name]

def astar_search(start_node, goal_node, graph):
    open_set = set()
    closed_set = set()
    
    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.g + start_node.h
    open_set.add(start_node)

    while open_set:
        current_node = min(open_set, key=lambda node: node.f)

        if current_node == goal_node:
            return reconstruct_path(current_node)

        open_set.remove(current_node)
        closed_set.add(current_node)

        for neighbor in graph.get_neighbors(current_node):
            neighbor_node = graph.nodes[neighbor]
            route = graph.get_route(current_node, neighbor_node)
            
            total_cost = route.calculate_total_cost()
            total_time = route.calculate_total_time()
            tentative_g = current_node.g + total_cost

            if tentative_g < neighbor_node.g:
                neighbor_node.parent = current_node
                neighbor_node.g = tentative_g
                neighbor_node.h = heuristic(neighbor_node, goal_node)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

            if neighbor_node not in open_set:
                open_set.add(neighbor_node)

    return None

def heuristic(node1, node2):
    return haversine(node1.location, node2.location)

def reconstruct_path(node):
    path = []
    while node:
        path.append(node)
        node = node.parent
    return path[::-1]

def find_nearest_stations(home_location, hotel_location, stations):
    nearest_from_home = sorted(stations, key=lambda station: haversine(home_location, station.location))[:3]
    nearest_to_hotel = sorted(stations, key=lambda station: haversine(hotel_location, station.location))[:3]
    
    return nearest_from_home, nearest_to_hotel

def collect_route_details(routes):
    total_time = 0
    total_cost = 0
    total_transfers = len(routes) - 1
    total_transfer_time = sum([route.transfer_time for route in routes])

    for route in routes:
        total_cost += route.calculate_total_cost()
        total_time += route.calculate_total_time()
    
    return total_time, total_cost, total_transfers, total_transfer_time

def plan_trip(home_location, hotel_location, stations, graph):
    nearest_from_home, nearest_to_hotel = find_nearest_stations(home_location, hotel_location, stations)
    
    all_routes = []

    for start_station in nearest_from_home:
        for end_station in nearest_to_hotel:
            route_to_station = astar_search(graph.nodes[start_station.name], graph.nodes[end_station.name], graph)
            
            if route_to_station:
                transport_routes = []
                for i in range(len(route_to_station) - 1):
                    transport_route = graph.get_route(route_to_station[i], route_to_station[i + 1])
                    transport_routes.append(transport_route)
                all_routes.append(transport_routes)

    detailed_routes = []
    
    for route in all_routes:
        time, cost, transfers, transfer_time = collect_route_details(route)
        detailed_routes.append({
            'total_time': time,
            'total_cost': cost,
            'total_transfers': transfers,
            'total_transfer_time': transfer_time,
            'route': route
        })

    return detailed_routes

if __name__ == "__main__":
    graph = Graph()
    graph.add_node("Station A", (40.7128, -74.0060))
    graph.add_node("Station B", (34.0522, -118.2437))
    graph.add_node("Station C", (51.5074, -0.1278))
    
    graph.add_edge("Station A", "Station B", 
                   TransportRoute(name="Train A-B", mode="train", class_type="economy", base_cost=100, base_time=6, 
                                  average_delay=1, transfer_time=0, stop_time=1, expected_waiting=0))
    graph.add_edge("Station B", "Station C", 
                   TransportRoute(name="Train B-C", mode="train", class_type="economy", base_cost=150, base_time=5, 
                                  average_delay=0, transfer_time=1, stop_time=1, expected_waiting=0))
    graph.add_edge("Station A", "Station C", 
                   TransportRoute(name="Flight A-C", mode="plane", class_type="business", base_cost=300, base_time=10, 
                                  average_delay=2, transfer_time=0, stop_time=0, expected_waiting=1))

    home_location = (40.730610, -73.935242)
    hotel_location = (51.5155, -0.0922)
    
    routes = plan_trip(home_location, hotel_location, graph.nodes.values(), graph)

    sorted_routes = sorted(routes, key=lambda x: (x['total_time'], x['total_cost'], x['total_transfers'], x['total_transfer_time']))
    
    for i, route_info in enumerate(sorted_routes):
        print(f"Маршрут {i + 1}:")
        print(f"    Общее время: {route_info['total_time']} часов")
        print(f"    Общая стоимость: {route_info['total_cost']} долларов")
        print(f"    Количество пересадок: {route_info['total_transfers']}")
        print(f"    Время пересадок: {route_info['total_transfer_time']} часов")
