from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import pandas as pd
import numpy as np
from math import radians, sin, cos, sqrt, atan2


def distance(lat1, lat2, lon1, lon2):
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371.0
    # calculate the result
    distance = r * c
    return distance


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    time_dimension = routing.GetDimensionOrDie('Time')
    total_distance = 0
    total_load = 0
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load = route_load + data['demands'][node_index]
            time_var = time_dimension.CumulVar(index)
            plan_output += ' {0} Load({1}) Time({2},{3}) -> '.format(node_index, route_load,
                                                                     solution.Min(time_var), solution.Max(time_var))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        time_var = time_dimension.CumulVar(index)
        plan_output += ' {0} Load({1}) Time({2},{3})\n'.format(manager.IndexToNode(index), route_load,
                                                               solution.Min(time_var), solution.Max(time_var))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        plan_output += 'Time of the route: {}min\n'.format(solution.Min(time_var))
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        total_time += solution.Min(time_var)
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    print('Total time of all routes: {}min'.format(total_time))


def main():
    """Entry point of the program."""
    df = pd.read_csv(r'C:\Users\Piyush\Downloads\BangaloreCSV_demand_100.csv')
    length = len(df)
    print(length)
    num_vehicles = 20
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
    # print(distance_matrix)
    # print(distance_matrix[289][0])

    speed = 5  # 5 KMPH
    time_matrix = distance_matrix / speed
    # print(time_matrix)

    time_windows = np.zeros((length - 1, 2))
    for i in range(length - 1):
        time_windows[i][0] = 0
        time_windows[i][1] = 1000
    time_windows = time_windows.astype(int)
    # print(time_windows)
    print('shape of time_windows', time_windows.shape)

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


    def create_data_model():
        """Stores the data for the problem."""
        data = {}
        data['distance_matrix'] = distance_matrix.tolist()
        data['time_matrix'] = time_matrix.tolist()
        data['time_windows'] = time_windows.tolist()
        data['pickups_deliveries'] = pickup_deliveries.tolist()
        data['demands'] = demand
        data['num_vehicles'] = 20
        data['vehicle_capacities'] = [20 * i / i for i in range(1, num_vehicles+1)]
        data['depot'] = (2 * length) - 1
        return data

    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        300,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <=
            time_dimension.CumulVar(delivery_index))

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
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    capacity_dimension.SetGlobalSpanCostCoefficient(100)
    # First Solution Heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 900

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    print("status", routing.status())


if __name__ == '__main__':
    main()
