from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import pandas as pd
import numpy as np
import random
from math import radians, sin, cos, asin, sqrt, atan2


def distance(lat1, lat2, lon1, lon2):

    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2*atan2(sqrt(a), sqrt(1 - a))

    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371.0
    # calculate the result
    distance = r*c
    return distance


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load = route_load + data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            # route_load = route_load - data['demands'][node_index]
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index), route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Entry point of the program."""
    df = pd.read_csv(r'BangaloreCSV_demand.csv')
    length = len(df)
    latitude1 = []
    longitude1 = []
    for i in range(len(df)):
        latitude = df['Pickup Latitude'][i]
        longitude = df['Pickup Longitude'][i]
        latitude1.append(latitude)
        longitude1.append(longitude)

    for j in range(len(df)):
        dlatitude = df['Drop Latitude'][j]
        dlongitude = df['Drop Longitude'][j]
        latitude1.append(dlatitude)
        longitude1.append(dlongitude)
    # print(latitude1)
    # print(longitude1)

    lat_array = np.array(latitude1)
    lon_array = np.array(longitude1)
    distance_matrix = []
    for k in range(0, 2 * len(df)):
        u = lat_array[k]
        x = lon_array[k]
        for m in range(0, 2 * len(df)):
            v = lat_array[m]
            y = lon_array[m]
            distance_matrix.append(round(distance(u, v, x, y)) * 10)
    distance_matrix = np.array(distance_matrix).reshape((len(lat_array), len(lat_array)))
    # print(distance_matrix.shape)
    # print(distance_matrix)
    distance_matrix = distance_matrix.astype(int)
    # print(distance_matrix)
    # print(distance_matrix[289][0])


    pickup_deliveries = np.zeros((length - 1, 2))
    for i in range(length - 1):
        pickup_deliveries[i][0] = int(i * 2)
        pickup_deliveries[i][1] = int(2 * i + 1)
    pickup_deliveries = pickup_deliveries.astype(int)
    # print(pickup_deliveries)
    # print(pickup_deliveries[143][0])

    demand = []
    for i in range(length + 1):
        b = 5
        if i < length - 1:
            demand.append(b)
            demand.append(-b)
        else:
            demand.append(0)
    demand = np.array(demand)
    # print(demand.shape)
    # print(demand)
    # print(demand[289])
    # print(length)


    def create_data_model():
        """Stores the data for the problem."""
        data = {}
        data['distance_matrix'] = distance_matrix.tolist()
        data['pickups_deliveries'] = pickup_deliveries.tolist()
        data['demands'] = demand
        data['num_vehicles'] = 20
        data['vehicle_capacities'] = [10*i/i for i in range(1, 21)]
        data['depot'] = (2*length)-1
        return data

    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index) <=
            distance_dimension.CumulVar(delivery_index))

    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    capacity_dimension = routing.GetDimensionOrDie(dimension_name)
    capacity_dimension.SetGlobalSpanCostCoefficient(100)
    # First Solution Heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    search_parameters.time_limit.seconds = 100

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    print("status", routing.status())


if __name__ == '__main__':
    main()
