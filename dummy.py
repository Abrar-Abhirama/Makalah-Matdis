import numpy as np
import matplotlib.pyplot as plt
from itertools import permutations
import random

class DeliveryTSP:
    def __init__(self, num_locations=10):
        self.locations = self.generate_locations(num_locations)
        self.distance_matrix = self.calculate_distance_matrix()
        
    def generate_locations(self, num_locations):
        # First location is depot (0,0)
        locations = [(0,0)]
        for _ in range(num_locations-1):
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            locations.append((x,y))
        return locations
    
    def calculate_distance_matrix(self):
        n = len(self.locations)
        matrix = np.zeros((n,n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    x1,y1 = self.locations[i]
                    x2,y2 = self.locations[j]
                    matrix[i][j] = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return matrix
    
    def solve_tsp_brute_force(self):
        n = len(self.locations)
        cities = list(range(1,n))  
        shortest_route = None
        min_distance = float('inf')
        
        for perm in permutations(cities):
            route = (0,) + perm + (0,)  
            distance = self.calculate_route_distance(route)
            if distance < min_distance:
                min_distance = distance
                shortest_route = route
                
        return shortest_route, min_distance
    
    def calculate_route_distance(self, route):
        return sum(self.distance_matrix[route[i]][route[i+1]] 
                  for i in range(len(route)-1))
    
    def visualize_route(self, route):
        plt.figure(figsize=(10,10))
    
        xs, ys = zip(*self.locations)
        plt.scatter(xs[1:], ys[1:], c='blue', label='Delivery Locations')
        plt.scatter(xs[0], ys[0], c='red', marker='s', label='Depot')
        
        for i in range(len(route)-1):
            start = self.locations[route[i]]
            end = self.locations[route[i+1]]
            plt.plot([start[0], end[0]], [start[1], end[1]], 'g--')
        
        plt.title('Delivery Route Optimization')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

# Example usage
if __name__ == "__main__":
    tsp = DeliveryTSP(10)
    
    route, distance = tsp.solve_tsp_brute_force()
    
    print(f"Best route: {route}")
    print(f"Total distance: {distance:.2f}")
    
    # Visualize solution
    tsp.visualize_route(route)