from itertools import permutations
import numpy as np
import matplotlib.pyplot as plt


class DeliveryTSP:
    def __init__(self, tsp_file_path):
        self.locations = self.load_tsplib_file(tsp_file_path)
        self.distance_matrix = self.calculate_distance_matrix()

    def load_tsplib_file(self, file_path):
        locations = []
        node_coord_section = False
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line == "NODE_COORD_SECTION":
                    node_coord_section = True
                    continue
                elif line == "EOF":
                    break
                if node_coord_section:
                    parts = line.split()
                    if len(parts) == 3:
                        x = float(parts[1])
                        y = float(parts[2])
                        locations.append((x, y))
        return locations

    def calculate_distance_matrix(self):
        n = len(self.locations)
        matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    x1, y1 = self.locations[i]
                    x2, y2 = self.locations[j]
                    matrix[i][j] = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return matrix

    def solve_tsp_nearest_neighbor(self):
        n = len(self.locations)
        visited = [False] * n
        route = [0]
        visited[0] = True
        total_distance = 0

        current = 0
        for _ in range(n - 1):
            nearest = None
            min_distance = float('inf')
            for next_city in range(n):
                if not visited[next_city] and self.distance_matrix[current][next_city] < min_distance:
                    nearest = next_city
                    min_distance = self.distance_matrix[current][next_city]
            route.append(nearest)
            visited[nearest] = True
            total_distance += min_distance
            current = nearest

        # Return to depot
        route.append(0)
        total_distance += self.distance_matrix[current][0]

        return route, total_distance

    def visualize_route(self, route):
        plt.figure(figsize=(12, 12))
        xs, ys = zip(*self.locations)
        plt.scatter(xs[1:], ys[1:], c='blue', label='Delivery Locations')
        plt.scatter(xs[0], ys[0], c='red', marker='s', label='Depot')
        for i in range(len(route) - 1):
            start = self.locations[route[i]]
            end = self.locations[route[i + 1]]
            plt.plot([start[0], end[0]], [start[1], end[1]], 'g--')
        plt.title('TSP Route Optimization')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()


if __name__ == "__main__":
    tsp = DeliveryTSP("berlin52.tsp")
    route, distance = tsp.solve_tsp_nearest_neighbor()
    print(f"Nearest Neighbor Route: {route}")
    print(f"Total Distance: {distance:.2f}")
    tsp.visualize_route(route)
