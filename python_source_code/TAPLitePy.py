# -*- coding: utf-8 -*-
"""
PyTAPLite is the Python deport of TAPLite.

Author: Wuyang Yuan
"""

import csv
import math
import pandas as pd
import osm2gmns as og
import numpy as np
import time
from datetime import datetime

# region global Const Variables
MAX_MODE_TYPES = 10
BIG_M = 9999999
INVALID = -1
DEBUG = 1


# endregion

# region global functions
def safe_float_conversion(s):
    if s == '':
        return 0.0  # or any default value you prefer
    try:
        return float(s)
    except ValueError:
        return 0.0  # handle cases where conversion fails


def alloc_1d(size, default_value=0.0):
    """
    Allocate a 1D array initialized with a default value.

    :param size: Size of the array.
    :param default_value: Value to initialize the array with.
    :return: NumPy 1D array.
    """
    return np.full(size + 1, default_value, dtype=float)  # +1 to mimic C++ behavior


def alloc_1d_int(size, default_value=0.0):
    """
    Allocate a 1D array initialized with a default value.

    :param size: Size of the array.
    :param default_value: Value to initialize the array with.
    :return: NumPy 1D array.
    """
    return np.full(size + 1, default_value, dtype=int)  # +1 to mimic C++ behavior


def alloc_2d(rows, cols, default_value=0.0):
    """
    Allocate a 2D array initialized with a default value.

    :param rows: Number of rows in the array.
    :param cols: Number of columns in the array.
    :param default_value: Value to initialize the array with.
    :return: NumPy 2D array.
    """
    return np.full((rows + 1, cols + 1), default_value, dtype=float)  # +1 to mimic C++ behavior


def alloc_2d_int(rows, cols, default_value=0.0):
    """
    Allocate a 2D array initialized with a default value.

    :param rows: Number of rows in the array.
    :param cols: Number of columns in the array.
    :param default_value: Value to initialize the array with.
    :return: NumPy 2D array.
    """
    return np.full((rows + 1, cols + 1), default_value, dtype=int)  # +1 to mimic C++ behavior


def alloc_3d(shape, default_value=0.0):
    """
    Allocate a 3D array with the given shape, initialized with a default value.

    :param shape: Tuple specifying the dimensions (e.g., (dim1, dim2, dim3)).
    :param default_value: Value to initialize the array with.
    :return: NumPy 3D array.
    """
    return np.full(shape, default_value, dtype=float)


def alloc_3d_int(shape, default_value=0.0):
    """
    Allocate a 3D array with the given shape, initialized with a default value.

    :param shape: Tuple specifying the dimensions (e.g., (dim1, dim2, dim3)).
    :param default_value: Value to initialize the array with.
    :return: NumPy 3D array.
    """
    return np.full(shape, default_value, dtype=int)


def print_log(str, level):
    if level <= DEBUG:
        print(str)


# endregion


class Node:
    def __init__(self):
        self.id = -1
        self.x = 0
        self.y = 0
        self.incoming_link_seq_no_vector = []
        self.outgoing_link_seq_no_vector = []


class Link:
    def __init__(self):
        self.internal_from_node_id = 0
        self.internal_to_node_id = 0
        self.link_id = 0
        self.link_type = 1
        self.external_from_node_id = 0
        self.external_to_node_id = 0

        self.Lane_Capacity = 0.0
        self.Link_Capacity = 0.0
        self.lanes = 1.0
        self.FreeTravelTime = 0.0
        self.free_speed = 10.0
        self.Cutoff_Speed = 0.0

        self.VDF_type = 0
        self.VDF_Alpha = 0.15
        self.VDF_Beta = 4
        self.VDF_plf = 1.0
        self.Q_cd = 1.0
        self.Q_n = 1.0
        self.Q_cp = 0.28125
        self.Q_s = 4.0

        self.length = 0.0
        self.Speed = 0.0
        self.allowed_uses = "all"

        self.mode_allowed_use = [0] * MAX_MODE_TYPES
        self.mode_MainVolume = [0.0] * MAX_MODE_TYPES

        self.mode_SubVolume = [0.0] * MAX_MODE_TYPES
        self.mode_SDVolume = [0.0] * MAX_MODE_TYPES

        self.mode_Toll = [0.0] * MAX_MODE_TYPES
        self.mode_AdditionalCost = [0.0] * MAX_MODE_TYPES

        self.travel_time = 0.0
        self.BPR_TT = 0.0
        self.QVDF_TT = 0.0

        self.GenCost = 0.0
        self.GenCostDer = 0.0
        self.Ref_volume = 0.0
        self.Base_volume = 0.0
        self.Obs_volume = -1.0
        self.geometry = ""

    def setup(self, num_of_modes):
        self.link_type = 1
        self.VDF_Alpha = 0.15
        self.VDF_Beta = 4.00
        self.VDF_plf = 1.0
        self.VDF_type = 0
        self.Q_cd = 1.0
        self.Q_n = 1.0
        self.Q_cp = 0.28125
        self.Q_s = 4.0
        self.travel_time = 0.0
        self.BPR_TT = 0.0
        self.QVDF_TT = 0.0
        self.Ref_volume = 0.0
        self.Base_volume = 0.0
        self.Obs_volume = -1.0


class Network:
    def __init__(self):
        self.links = []  # List of links
        self.md_od_flow = None  # List of o-d-double-indexed flows
        self.total_o_flow = None  # List of o-indexed flows
        self.md_route_cost = None  # List of route cost
        self.map_node_seq_no_2_external_node_id = {}  # Maps node sequence number to external node ID
        self.map_external_node_id_2_node_seq_no = {}  # Maps external node ID to node sequence number
        self.map_external_node_id_2_zone_id = {}  # Maps external node ID to zone_id
        self.zone_outbound_link_size = []
        self.first_link_from = []
        self.last_link_from = []
        self.link_indices = None
        self.number_of_nodes = 0
        self.number_of_links = 0
        self.number_of_zones = 0
        self.number_of_modes = 1
        self.first_thru_node = 0


class NetworkReader:
    def __init__(self):
        self.network = Network()
        self.log_file = None  # Path to the log file, if any.
        self.node_file_name = 'node.csv'  # Path to the node file.
        self.link_file_name = 'link.csv'  # Path to the link file.
        self.demand_file_name = 'demand.csv'  # Path to the demand file.
        self.use_us_standard = False

    def process_node_file(self):
        # Note: we do not store the node in memory

        tap_log_enabled = self.log_file is not None

        # Initialize variables
        number_of_nodes = 0
        number_of_zones = 0
        first_thru_node = 0
        l_first_thru_node = 1

        # Open the log file if provided
        logfile = open(self.log_file, 'w') if self.log_file else None

        # Open and read the CSV file
        with open(self.node_file_name, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                # Read node_id and zone_id
                node_id = int(row.get("node_id", 0))
                # Handle missing or empty zone_id values
                zone_id_value = row.get("zone_id", "")
                zone_id = int(zone_id_value) if zone_id_value.isdigit() else 0

                # Error check: zone_id should match node_id
                if zone_id >= 1 and zone_id != node_id:
                    print(f"Error: zone_id should be the same as node_id but zone_id = {zone_id}, node_id = {node_id}")

                # Update mappings
                self.network.map_node_seq_no_2_external_node_id[number_of_nodes + 1] = node_id
                self.network.map_external_node_id_2_node_seq_no[node_id] = number_of_nodes + 1  # Sequential number starts from 1
                self.network.map_external_node_id_2_zone_id[node_id] = zone_id

                # Update number_of_zones
                if zone_id >= 1 and zone_id > number_of_zones:
                    number_of_zones = zone_id

                # Set first through node if not initialized
                if zone_id == 0 and l_first_thru_node == first_thru_node:  # Not initialized
                    l_first_thru_node = number_of_nodes + 1

                # Log data if tap_log_file is enabled
                if tap_log_enabled:
                    logfile.write(f"node_id = {node_id}, node_seq_no = {self.network.map_external_node_id_2_node_seq_no[node_id]}\n")

                # Increment node count
                number_of_nodes += 1

        # Close the log file if it was opened
        if logfile:
            logfile.close()

        print(self.network.map_node_seq_no_2_external_node_id)

        self.network.number_of_nodes = number_of_nodes
        self.network.number_of_zones = number_of_zones
        self.network.first_thru_node = l_first_thru_node
        return

    def get_number_of_links(self):
        number_of_links = 0

        # Open and read the CSV file
        with open(self.link_file_name, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                # Read link_id and from_node_id (values are not used in counting)
                link_id = int(row.get("link_id", 0))
                from_node_id = int(row.get("from_node_id", 0))

                # Increment the link count
                number_of_links += 1

        self.network.number_of_links = number_of_links
        return number_of_links

    def read_links(self):
        total_base_link_volume = 0
        k = 1  # Link index starts from 1

        with open(self.link_file_name, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            number_of_modes = 1  # TODO: get the value of number_of_modes from outer scope

            for row in csv_reader:
                link = Link()
                link.setup(number_of_modes)

                # Read basic properties
                link.external_from_node_id = int(row["from_node_id"])
                link.external_to_node_id = int(row["to_node_id"])
                link.link_id = int(row["link_id"])
                link.link_type = int(row["link_type"])

                # Map internal node IDs
                link.internal_from_node_id = self.network.map_external_node_id_2_node_seq_no.get(link.external_from_node_id, 0)
                if link.internal_from_node_id == 0:
                    print(f"Error in from_node_id = {link.external_from_node_id} for link_id = {link.link_id}")
                    continue

                link.internal_to_node_id = self.network.map_external_node_id_2_node_seq_no.get(link.external_to_node_id, 0)
                if link.internal_to_node_id == 0:
                    print(f"Error in to_node_id = {link.external_to_node_id} for link_id = {link.link_id}")
                    continue

                #  Set use_us_standard to True for US Standard (miles, mph), False for International (meters, kmph)

                link.Ref_volume = safe_float_conversion(row.get("ref_volume", 0.0))
                link.length = safe_float_conversion(row.get("length", 0.0))

                if "lanes" in row:
                    link.lanes = int(row["lanes"])
                if "capacity" in row:
                    link.Lane_Capacity = safe_float_conversion(row["capacity"])
                    link.Link_Capacity = link.lanes * link.Lane_Capacity

                link.free_speed = safe_float_conversion(row.get("free_speed", 10.0))

                if 'vdf_type' in row:
                    link.VDF_type = int(row['vdf_type'])
                if 'vdf_alpha' in row:
                    link.VDF_Alpha = float(row['vdf_alpha'])
                if 'vdf_beta' in row:
                    link.VDF_Beta = float(row['vdf_beta'])

                # Read additional properties
                # Determine the unit for length based on the standard
                if self.use_us_standard:
                    # Assume length is in miles, speed in mph
                    pass  # No conversion needed
                else:
                    # Length is in meters (International Standard), speed in kmph
                    link.length /= 1609.34  # convert to mile
                    link.free_speed /= 1.609  # convert to mile per hour

                link.FreeTravelTime = link.length / link.free_speed * 60.0

                # Debugging output
                if DEBUG and k < 3:
                    print("Processing link {k} properties:")
                    print(f"  Length: {link.length}")
                    print(f"  Lanes: {link.lanes}")
                    print(f"  Lane Capacity: {link.Lane_Capacity}")
                    print(f"  Link Capacity: {link.Link_Capacity}")
                    print(f"  Free Speed: {link.free_speed}")
                    print(f"  Free Travel Time: {link.FreeTravelTime}")

                # # Handle mode-based properties
                # for m in range(1, number_of_modes + 1):
                #     mode_field = f"base_vol_{g_mode_type_vector[m]['mode_type']}"

                #     toll_field = f"toll_{g_mode_type_vector[m]['mode_type']}"
                #     if toll_field in row:
                #         link.mode_Toll[m] = float(row.get(toll_field, 0.0))
                #         link.mode_AdditionalCost[m] = link.mode_Toll[m] / g_mode_type_vector[m]["vot"] * 60.0

                # Final processing
                link.BoverC = link.VDF_Alpha / pow(link.Link_Capacity, link.VDF_Beta) if link.Link_Capacity > 0 else 0.0

                self.network.links[k] = link
                k += 1

        print(f"total_base_link_volume = {total_base_link_volume}")
        baselinkvolume_loaded_flag = 1 if total_base_link_volume > 0 else 0

        return

    def find_links_to(self):
        # Initialize a dictionary with empty lists for each node
        links_to = {node: [] for node in range(1, self.network.number_of_nodes + 1)}

        # Populate the links_to dictionary
        for k in range(1, self.network.number_of_links + 1):
            internal_to_node_id = self.network.links[k].internal_to_node_id
            links_to[internal_to_node_id].append(k)

        # Sort the lists for each node
        for node in links_to:
            links_to[node].sort()

        return links_to

    def init_link_pointers(self):
        # Initialize FirstLinkFrom and LastLinkFrom lists
        self.network.first_link_from = [0] * (self.network.number_of_nodes + 1)  # 1-based indexing
        self.network.last_link_from = [-1] * (self.network.number_of_nodes + 1)  # -1 indicates no links

        self.network.first_link_from[1] = 1
        Node = 1

        for k in range(1, self.network.number_of_links + 1):
            internal_from_node_id = self.network.links[k].internal_from_node_id

            if internal_from_node_id == Node:
                continue
            elif internal_from_node_id >= Node + 1:
                self.network.last_link_from[Node] = k - 1
                Node = internal_from_node_id
                self.network.first_link_from[Node] = k
            elif internal_from_node_id < Node:
                raise ValueError(f"Sort error in link file '{self.link_file_name}': "
                                 f"a link from node {internal_from_node_id} was found after a link from node {Node}.")
            elif internal_from_node_id > Node + 1:
                # Handle nodes with no links
                self.network.last_link_from[Node] = k - 1
                for Node in range(Node + 1, internal_from_node_id):
                    self.network.first_link_from[Node] = 0
                    self.network.last_link_from[Node] = -1
                self.network.first_link_from[Node] = k

        if Node == self.network.number_of_nodes:
            self.network.last_link_from[Node] = self.network.number_of_links
        else:
            # Handle remaining nodes with no links
            self.network.last_link_from[Node] = self.network.number_of_links - 1
            for Node in range(Node + 1, self.network.number_of_nodes + 1):
                self.network.first_link_from[Node] = 0
                self.network.last_link_from[Node] = -1

        # Optional logging
        if self.log_file == 1:  # and self.logfile_path:
            with open(self.log_file, 'w') as logfile:
                for Node in range(1, self.network.number_of_nodes + 1):
                    logfile.write(f"node_id = {self.network.map_node_seq_no_2_external_node_id.get(Node, Node)}, "
                                  f"FirstLinkFrom = {self.network.first_link_from[Node]}, "
                                  f"LastLinkFrom = {self.network.last_link_from[Node]} \n")

        return

    def initialize_link_indices(self):
        max_routes = 10
        """Initialize the global 5D list for link sequences."""
        start_time = time.time()

        # Create a 5D list filled with empty lists
        self.network.link_indices = [
            [
                [
                    [[] for _ in range(max_routes + 1)]
                    for _ in range(self.network.number_of_zones + 1)
                ]
                for _ in range(self.network.number_of_zones + 1)
            ]
            for _ in range(self.network.number_of_modes + 1)
        ]

        end_time = time.time()
        duration = end_time - start_time

        hours, rem = divmod(duration, 3600)
        minutes, seconds = divmod(rem, 60)
        milliseconds = (seconds - int(seconds)) * 1000
        seconds = int(seconds)

        print(f"Memory creation time for 5D link path matrix: {int(hours)} hours {int(minutes)} minutes {seconds} seconds {int(milliseconds)} ms")

    def read_od_flow(self):
        """
        Read the OD flow and initialize required data structures.
        """
        od_table_shape = (self.network.number_of_modes + 1, self.network.number_of_zones + 1, self.network.number_of_zones + 1)
        self.network.md_od_flow = alloc_3d(od_table_shape)
        self.network.md_route_cost = alloc_3d(od_table_shape)
        self.network.total_o_flow = alloc_1d(self.network.number_of_zones + 1)

        m = 1
        for orig in range(1, self.network.number_of_zones + 1):
            self.network.total_o_flow[orig] = 0
            for dest in range(1, self.network.number_of_zones + 1):
                self.network.md_od_flow[m][orig][dest] = 0
                self.network.md_route_cost[m][orig][dest] = BIG_M

        # Populate MDODflow
        self.read_od_table()

        # Calculate the total OD flow
        real_total = self.sum_od_table()

        # Calculate outbound link size
        self.network.zone_outbound_link_size = alloc_1d(self.network.number_of_zones + 1, default_value=0)
        for k in range(1, self.network.number_of_links + 1):
            if self.network.links[k].external_from_node_id <= self.network.number_of_zones:
                self.network.zone_outbound_link_size[self.network.links[k].external_from_node_id] += 1

        # Error checking
        for z in range(1, self.network.number_of_zones + 1):
            if self.network.zone_outbound_link_size[z] == 0 and self.network.total_o_flow[z] > 0.01:
                raise ValueError(f"No outbound link from zone {z} with positive demand {self.network.total_o_flow[z]}")

        return

    def read_od_table(self):
        """
        Read OD table and populate OD and DiffOD tables.
        """
        m = 1
        line_count = 0

        try:
            with open(self.demand_file_name, 'r', encoding='utf-8-sig') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    o_zone_id = int(row['o_zone_id'])
                    d_zone_id = int(row['d_zone_id'])
                    volume = float(row['volume'])

                    if o_zone_id > self.network.number_of_zones or d_zone_id > self.network.number_of_zones:
                        raise ValueError(f"Invalid zone ID in {self.demand_file_name}: {o_zone_id}, {d_zone_id}")

                    if line_count <= 3:
                        print(f"o_zone_id: {o_zone_id}, d_zone_id: {d_zone_id}, volume: {volume:.4f}")
                        line_count = line_count + 1

                    self.network.md_od_flow[m, o_zone_id, d_zone_id] = volume
                    print(self.network.md_od_flow[m, o_zone_id, d_zone_id])

        except FileNotFoundError:
            print(f"Error: File {self.demand_file_name} not found.")
            return

    def sum_od_table(self):
        """
        Calculate the sum of the OD table and populate total_o_table.

        """
        total_sum = 0
        for m in range(1, self.network.number_of_modes + 1):
            for orig in range(1, self.network.number_of_zones + 1):
                for dest in range(1, self.network.number_of_zones + 1):
                    self.network.total_o_flow[orig] += self.network.md_od_flow[m, orig, dest]
                    total_sum += self.network.md_od_flow[m, orig, dest]

        # Print the OD sum volume for each zone
        print("OD Sum Volume for Each Zone:")
        for orig in range(1, self.network.number_of_zones + 1):
            # Sum all destination flows for a given origin and mode
            zone_sum = self.network.md_od_flow[1, orig, :].sum()  # Sums all destinations for mode 1 and origin `orig`
            print(f"Zone {orig}: {zone_sum:.4f}")

        # Print the total OD volume
        print(f"Total OD Volume Across All Zones: {total_sum:.4f}")

        return total_sum

    def read_data(self):
        # Step 0: Process Nodes
        self.process_node_file()

        # Step 0.1: Detecting the num of links.
        self.get_number_of_links()
        self.network.links = [None] * (self.network.number_of_links + 1)  # +1 for 1-based indexing

        # Step 1: Read links from file
        self.read_links()
        print(f" after reading link.csv, Length of Link: {len(self.network.links)}")

        # Step 2: Find 'LinksTo' structure
        links_to = self.find_links_to()

        # Step 3: Initialize link pointers
        self.init_link_pointers()

        # Step 4: Initialize the global 5D list for link sequences.
        self.initialize_link_indices()

        # Step 5: Read demand
        self.read_od_flow()


class FlowAssignmentSolver:
    def __init__(self, network: Network):
        self.max_assign_iterations = 30
        self.network = network
        self.demand_period_starting_hours = 7
        self.demand_period_ending_hours = 8
        self.output_file_accessibility = 'accessibility_matrix.csv'
        self.output_file_route_assignment = 'route_assignment.csv'
        self.output_file_link_performance = 'link_performance.csv'

    def solve(self):  # process assignment
        MainVolume = np.zeros(self.network.number_of_links + 1)
        SDVolume = np.zeros(self.network.number_of_links + 1)
        SubVolume = np.zeros(self.network.number_of_links + 1)
        MDMinPathPredLink = np.zeros((self.network.number_of_modes + 1, self.network.number_of_zones + 1, self.network.number_of_nodes + 1), dtype=int)

        for k in range(1, self.network.number_of_links + 1):
            MainVolume[k] = 0

        system_wide_travel_time = self.update_link_cost(MainVolume)
        system_least_travel_time = self.find_min_cost_routes(MDMinPathPredLink)
        self.output_accessibility_matrix()
        self.all_or_nothing_assign(0, MainVolume, MDMinPathPredLink)
        system_wide_travel_time = self.update_link_cost(MainVolume)

        for iteration_no in range(1, self.max_assign_iterations):
            system_least_travel_time = self.find_min_cost_routes(MDMinPathPredLink)
            self.all_or_nothing_assign(iteration_no, SubVolume, MDMinPathPredLink)  # assign to the subvolume
            self.volume_difference(SubVolume, MainVolume, SDVolume)
            lambda_ = self.links_sd_line_search(MainVolume, SDVolume)
            self.update_volume(MainVolume, SDVolume, lambda_)
            system_wide_travel_time = self.update_link_cost(MainVolume)
            gap = (system_wide_travel_time - system_least_travel_time) / max(0.1, system_least_travel_time) * 100
            print_log(f"Iter No = {iteration_no}, Lambda = {lambda_:.6f}, System VMT = {system_wide_travel_time:.1f}, Least TT = {system_least_travel_time:.1f}, Gap = {gap:.2f}%",1)

        self.output_route_details()
        self.output_link_performance()
        # Output
        print_log(f"System Least Travel Time: {system_least_travel_time:.2f}",1)  #

    # region Auxiliary Functions
    def update_link_cost(self, main_volume):  # update_link_cost_by_volume
        system_wide_travel_time = 0.0

        print_log(f"Length of Link: {len(self.network.links)}",2)

        for k in range(1, self.network.number_of_links + 1):  # Adjust range as needed
            # if k >= len(g_link_vector):
            #     print(f"Invalid index: {k} exceeds Link size {len(Link)}")

            # print(f"Link {k}: calling link_travel_time")
            self.network.links[k].travel_time = self.link_travel_time(k, main_volume)
            # print(f"Link {k}: Travel Time = {g_link_vector[k].travel_time}")

            self.network.links[k].GenCost = self.link_gen_cost(k, main_volume)
            # print(f"Link {k}: Generalized Cost = {g_link_vector[k].GenCost}")

            contribution_to_travel_time = main_volume[k] * self.network.links[k].travel_time
            system_wide_travel_time += contribution_to_travel_time
            # print(f"Link {k}: Contribution to System-Wide Travel Time = {contribution_to_travel_time}")

        print_log(f"Total System-Wide Travel Time = {system_wide_travel_time}",2)
        return system_wide_travel_time

    def link_gen_cost(self, k, volume):
        return self.network.links[k].mode_AdditionalCost[1] + self.link_travel_time(k, volume)

    def link_integral_cost(self, k, volume):
        # Warming: Not Implemented!!!

        if k >= len(self.network.links):
            print(f"Invalid index: {k} exceeds Link size {len(self.network.links)}")  # Modified

        incoming_demand = (
                volume[k]
                / max(0.01, self.network.links[k].lanes)
                / max(0.001, self.demand_period_ending_hours - self.demand_period_starting_hours)
                / max(0.0001, self.network.links[k].VDF_plf)
        )
        capacity = self.network.links[k].Link_Capacity
        x = incoming_demand / capacity

        integral = 0

        if self.network.links[k].VDF_type == 0:  # BPR
            capacity = self.network.links[k].Link_Capacity

            self.network.links[k].travel_time = (
                    self.network.links[k].FreeTravelTime
                    * (1.0 + self.network.links[k].VDF_Alpha * (incoming_demand / max(0.1, self.network.links[k].Link_Capacity)) ** self.network.links[k].VDF_Beta)
            )
        elif self.network.links[k].VDF_type == 1:  # Conical
            x = incoming_demand / capacity
            alpha = self.network.links[k].VDF_Alpha
            beta = (2 * alpha - 1) / (2 * alpha - 2)
            self.network.links[k].travel_time = self.network.links[k].FreeTravelTime * (2 + math.sqrt(alpha * alpha * (1 - x) * (1 - x) + beta * beta) - alpha * (1 - x) - beta)
        return integral

    def link_travel_time(self, k, volume):  # update the travel time of the k-th link
        # print(f"Processing Link {k}")

        if k >= len(self.network.links):
            print(f"Invalid index: {k} exceeds Link size {len(self.network.links)}")  # Modified

        incoming_demand = (
                volume[k]
                / max(0.01, self.network.links[k].lanes)
                / max(0.001, self.demand_period_ending_hours - self.demand_period_starting_hours)
                / max(0.0001, self.network.links[k].VDF_plf)
        )

        capacity = self.network.links[k].Link_Capacity
        x = incoming_demand / capacity
        print_log(f"Link {k}: Incoming Demand = {incoming_demand}", 2)

        if self.network.links[k].VDF_type == 0:  # BPR
            self.network.links[k].travel_time = (self.network.links[k].FreeTravelTime * (1.0 + self.network.links[k].VDF_Alpha * (x ** self.network.links[k].VDF_Beta)))
        elif self.network.links[k].VDF_type == 1:  # Conical
            alpha = self.network.links[k].VDF_Alpha
            beta = (2 * alpha - 1) / (2 * alpha - 2)
            self.network.links[k].travel_time = (self.network.links[k].FreeTravelTime * (2 + math.sqrt(alpha * alpha * (1 - x) * (1 - x) + beta * beta) - alpha * (1 - x) - beta))

        print_log(f"Link {k}: Calculated Travel Time = {self.network.links[k].travel_time}", 2)

        if self.network.links[k].travel_time < 0:
            self.network.links[k].travel_time = 0
            print_log(f"Link {k}: Travel Time adjusted to 0 (was negative)", 2)

        self.network.links[k].BPR_TT = self.network.links[k].travel_time
        print_log(f"Link {k}: BPR_TT = {self.network.links[k].BPR_TT}", 2)

        return self.network.links[k].travel_time

    # endregion

    # region Shortest Path Label-Correcting
    def find_min_cost_routes(self, min_path_pred_link):
        """
        Finds the minimum cost routes for all origins and modes.
        """
        cost_to = np.full((self.network.number_of_zones + 1, self.network.number_of_nodes + 1), BIG_M, dtype=float)

        system_least_travel_time = 0
        m = 1

        for orig in range(1, self.network.number_of_zones + 1):

            if orig not in self.network.map_external_node_id_2_zone_id:
                continue

            if self.network.map_external_node_id_2_zone_id[orig] < 1:
                continue

            self.minpath(m, orig, min_path_pred_link[m][orig], cost_to[orig])

            if self.network.md_route_cost is not None:
                for dest in range(1, self.network.number_of_zones + 1):
                    if dest not in self.network.map_external_node_id_2_zone_id:
                        continue

                    self.network.md_route_cost[m][orig][dest] = BIG_M

                    internal_node_id_for_destination_zone = self.network.map_external_node_id_2_node_seq_no[dest]
                    if cost_to[orig][internal_node_id_for_destination_zone] <= BIG_M - 1:
                        self.network.md_route_cost[m][orig][dest] = cost_to[orig][internal_node_id_for_destination_zone]
                        if self.network.md_od_flow[m][orig][dest] > 1e-6:
                            system_least_travel_time += self.network.md_route_cost[m][orig][dest] * self.network.md_od_flow[m][orig][dest]

                        if DEBUG >=2:
                            print(f"Processing mode {m}, origin {orig}")
                            print(f"    Destination {dest}:")
                            print(f"      Cost to Destination: {cost_to[orig][internal_node_id_for_destination_zone]:.4f}")
                            print(f"      Updated md_route_cost[{m}][{orig}][{dest}] = {self.network.md_route_cost[m][orig][dest]:.4f}")
                            print(f"      Updated System Least Travel Time: {system_least_travel_time:.4f}")

        return system_least_travel_time

    def minpath(self, mode, orig, pred_link, cost_to):
        """
        Computes the shortest path from an origin node using a modified label-setting algorithm.
        """
        cost_to.fill(BIG_M)
        pred_link.fill(INVALID)

        # Initialize variables
        queue_next = alloc_1d_int(self.network.number_of_nodes + 1)

        now = self.network.map_external_node_id_2_node_seq_no[orig]
        internal_node_id_for_origin_zone = now

        pred_link[now] = INVALID
        cost_to[now] = 0.0

        scan_list = []
        scan_list.append(now)
        return2q_count = 0

        if DEBUG >= 2:
            print(f"Starting minpath computation for mode {mode}, origin {orig}")
            print(f"Initial node: {now}, queue initialized")

        while scan_list:
            now = scan_list.pop(0)  # Remove the first element from the scan list
            if now >= self.network.first_thru_node or now == internal_node_id_for_origin_zone:
                if DEBUG>= 2:
                    print(f"Processing node {now}...")

                for k in range(self.network.first_link_from[now], self.network.last_link_from[now] + 1):
                    #    if g_link_vector[k]["mode_allowed_use"][mode] == 0:
                    #        continue

                    new_node = self.network.links[k].internal_to_node_id
                    new_cost = cost_to[now] + self.network.links[k].travel_time

                    if DEBUG>= 2:
                        print(f"Checking link {k}: new_node={new_node}, g_link_vector[k].length  = {self.network.links[k].length}, new_cost={new_cost:.4f}, "
                              f"current_cost={cost_to[new_node]:.4f}")

                    if cost_to[new_node] > new_cost:

                        if DEBUG>= 2:
                            print(f"Updated cost for node {new_node}: {new_cost:.4f}")
                            print(f"Predecessor for node {new_node}: link {k}")

                        cost_to[new_node] = new_cost
                        pred_link[new_node] = k

                        # Add the node to the scan list if it's not already there
                        if new_node not in scan_list:
                            scan_list.append(new_node)
                            if DEBUG>= 2:
                                print(f"    Node {new_node} added to scan list")

        if DEBUG >= 2:
            print("Updated cost_to array:")
            for idx, cost in enumerate(cost_to):
                print(f"  Node {idx}: Cost = {cost:.4f}")

            print("Updated pred_link array:")
            for idx, pred in enumerate(pred_link):
                print(f"  Node {idx}: Predecessor Link = {pred}")

        if DEBUG >= 2:
            print("Finished minpath computation")
            print(f"Total nodes returned to queue: {return2q_count}")

            return return2q_count

    # endregion

    # region Output the solving process
    def output_accessibility_matrix(self):
        # Open the CSV file for writing
        with open(self.output_file_accessibility, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)

            # Write the header row
            csv_writer.writerow(["o_zone_id", "d_zone_id", "mode_id", "cost"])

            # Loop through all origin zones
            for Orig in range(1, self.network.number_of_zones + 1):

                if Orig not in self.network.map_external_node_id_2_zone_id:
                    continue

                # Skip zones with no outbound links
                if self.network.zone_outbound_link_size[Orig] == 0:
                    continue

                # Iterate over all destinations
                for Dest in range(1, self.network.number_of_zones + 1):

                    if Dest not in self.network.map_external_node_id_2_zone_id:
                        continue

                    if Dest == Orig:
                        continue  # Skip self-loops

                    for m in range(1, self.network.number_of_modes + 1):  # Iterate over modes
                        # Skip if cost is not feasible (BIGM represents an infeasible cost)
                        if self.network.md_route_cost[m][Orig][Dest] >= BIG_M - 1:
                            continue

                        # Write the valid record to the CSV
                        csv_writer.writerow([Orig, Dest, m, self.network.md_route_cost[m][Orig][Dest]])

        print(f"Accessibility matrix has been successfully written to {self.output_file_accessibility}.")

    def output_route_details(self):
        """
        Write route details to a CSV file.

        Args:
            filename (str): The name of the output CSV file.
            g_link_indices (list): Nested list containing route information.
            link_data (list): List of link objects with attributes such as length, travel_time, etc.
            mode_type_vector (list): List of mode types with `mode_type` attributes.
        """
        # Open the file for writing
        with open(self.output_file_route_assignment, 'w', newline='') as output_file:
            writer = csv.writer(output_file)

            # Write the CSV header in lowercase
            writer.writerow([
                "mode", "route_seq_id", "o_zone_id", "d_zone_id", "unique_route_id",
                "node_sequence", "link_ids", "total_distance", "total_free_flow_travel_time",
                "total_travel_time", "route_key"
            ])

            if not self.network.link_indices:
                return  # Exit if no route data is available

            unique_route_id = 1
            # Loop through the modes
            for m in range(1, len(self.network.link_indices)):
                for orig in range(1, len(self.network.link_indices[m])):
                    for dest in range(1, len(self.network.link_indices[m][orig])):
                        unique_routes = {}

                        for route_id, route in enumerate(self.network.link_indices[m][orig][dest]):
                            if route:  # Check if the route is non-empty
                                total_distance = 0.0
                                total_free_flow_travel_time = 0.0
                                total_travel_time = 0.0
                                node_ids_str = ""
                                link_ids_str = ""

                                node_sum = 0  # Sum of node IDs
                                link_sum = 0  # Sum of link IDs

                                # Collect node IDs, link indices, and compute total metrics
                                for i in range(len(route) - 1, -1, -1):
                                    k = route[i]
                                    # print(f"route[{i}] = {k}, type: {type(k)}")  # Debugging print statement

                                    # Append the from_node_id for each link
                                    from_node_id = self.network.links[k].external_from_node_id
                                    node_ids_str += f"{from_node_id};"
                                    node_sum += from_node_id

                                    # Append the link index
                                    link_ids_str += f"{k};"
                                    link_sum += k

                                    # Sum up total distance and travel times
                                    total_distance += self.network.links[k].length
                                    total_free_flow_travel_time += self.network.links[k].FreeTravelTime
                                    total_travel_time += self.network.links[k].travel_time

                                    # Add the to_node_id for the last link
                                    if i == 0:
                                        to_node_id = self.network.links[k].external_to_node_id
                                        node_ids_str += f"{to_node_id}"
                                        node_sum += to_node_id

                                # Create a unique key based on node sum and link sum
                                route_key = f"{node_sum}_{link_sum}"

                                # Check for uniqueness
                                if route_key not in unique_routes:
                                    # Mark the route as unique
                                    unique_routes[route_key] = True

                                    # Remove trailing semicolon from link_ids_str
                                    if link_ids_str.endswith(";"):
                                        link_ids_str = link_ids_str[:-1]

                                    # Write the route data to the CSV file
                                    writer.writerow([
                                        m, route_id, orig, dest,
                                        unique_route_id, node_ids_str, link_ids_str,
                                        total_distance, total_free_flow_travel_time,
                                        total_travel_time, route_key
                                    ])

                                    unique_route_id = unique_route_id + 1

        print(f"Output written to {self.output_file_route_assignment}")

    def output_link_performance(self):
        """
        Write link performance data to a CSV file.

        Parameters:
            iteration_no: Current iteration number.
            main_volume: List of main volumes on links.
            links: List of link objects with attributes.
            mode_type_vector: List of mode type objects with attributes.
            demand_period_starting_hours: Start hour of the demand period.
            demand_period_ending_hours: End hour of the demand period.
        """
        # print("g_link_vector initialization:")
        # for link in g_link_vector:
        #    print(link)

        try:
            with open(self.output_file_link_performance, mode='w', newline='') as link_file:
                # Write headers
                writer = csv.writer(link_file)
                headers = ["link_id", "from_node_id", "to_node_id", "volume", "ref_volume",
                           "base_volume", "obs_volume", "capacity", "D", "doc", "fftt", "travel_time",
                           "VDF_alpha", "VDF_beta", "VDF_plf", "speed", "VMT", "VHT", "PMT", "PHT",
                           "VHT_QVDF", "PHT_QVDF", "geometry"
                           ]
                # Add mode-specific headers
                # headers.extend([f"mod_vol_{mode.mode_type}" for mode in mode_type_vector])
                headers.extend([
                    "P", "t0", "t2", "t3", "vt2", "mu", "Q_gamma", "free_speed", "cutoff_speed",
                    "congestion_ref_speed", "avg_queue_speed", "avg_QVDF_period_speed",
                    "avg_QVDF_period_travel_time", "Severe_Congestion_P"
                ])
                # Add speed intervals
                headers.extend(
                    [f"spd_{t // 60:02}:{t % 60:02}" for t in range(self.demand_period_starting_hours * 60, self.demand_period_ending_hours * 60, 5)]
                )
                writer.writerow(headers)

            # Append data
            with open(self.output_file_link_performance, mode='a', newline='') as link_file:
                writer = csv.writer(link_file)

                for k, link in enumerate(self.network.links):
                    # Placeholder values (replace with actual calculations)
                    if link is None:
                        print(f"Error: Link at index {k} is None.")
                        continue

                    if not hasattr(link, 'mode_MainVolume'):
                        print(f"Error: Link at index {k} is missing 'mode_MainVolume'.")
                        continue
                    P = t0 = t2 = t3 = vt2 = mu = Severe_Congestion_P = 0
                    Q_gamma = congestion_ref_speed = avg_queue_speed = avg_QVDF_period_speed = 0
                    IncomingDemand = DOC = 0

                    # Call your Link_QueueVDF function here
                    model_speed = [0] * 300  # Placeholder for model speed array

                    # Calculate VMT, VHT, PMT, PHT, VHT_QVDF, PHT_QVDF
                    VMT = VHT = PMT = PHT = VHT_QVDF = PHT_QVDF = 0
                    m = 1  # for m, mode in enumerate(mode_type_vector, start=1):
                    occ = 1

                    VMT += link.mode_MainVolume[m] * link.length
                    VHT += link.mode_MainVolume[m] * link.travel_time / 60.0
                    PMT += link.mode_MainVolume[m] * occ * link.length
                    PHT += link.mode_MainVolume[m] * occ * link.travel_time / 60.0
                    VHT_QVDF += link.mode_MainVolume[m] * link.QVDF_TT / 60.0
                    PHT_QVDF += link.mode_MainVolume[m] * occ * link.QVDF_TT / 60.0

                    # Create a row of data
                    row = [
                        link.link_id, link.external_from_node_id, link.external_to_node_id,
                        link.mode_MainVolume[m], link.Ref_volume, link.Base_volume, link.Obs_volume, link.Link_Capacity,
                        IncomingDemand, DOC, link.FreeTravelTime, link.travel_time, link.VDF_Alpha, link.VDF_Beta,
                        link.VDF_plf, link.length / max(link.travel_time / 60.0, 0.001),
                                      link.travel_time - link.FreeTravelTime, VMT, VHT, PMT, PHT, VHT_QVDF, PHT_QVDF,
                        link.geometry
                    ]

                    # Add mode-specific data m = 1
                    row.extend([link.mode_MainVolume[m]])

                    # Add additional parameters
                    row.extend([
                        P, t0, t2, t3, vt2, mu, Q_gamma, link.free_speed, link.Cutoff_Speed,
                        congestion_ref_speed, avg_queue_speed, avg_QVDF_period_speed, link.QVDF_TT, Severe_Congestion_P
                    ])

                    # Add speed data
                    row.extend([model_speed[t // 5] for t in range(self.demand_period_starting_hours * 60, self.demand_period_ending_hours * 60, 5)])

                    # Write the row to the file
                    writer.writerow(row)
        except Exception as e:
            print(f"Error: {e}")

    # endregion

    def all_or_nothing_assign(self, assignment_iteration_no, main_volume, MDMinPathPredLink):
        print_log(f"All or nothing assignment, assignment_iteration_no = {assignment_iteration_no}",2)
        # Initialize ProcessorVolume and ProcessorModeVolume
        Volume = np.zeros((self.network.number_of_links + 1))
        ModeVolume = np.zeros((self.network.number_of_links + 1, self.network.number_of_modes + 1))

        for Orig in range(1, self.network.number_of_zones + 1):
            # Skip zones with no positive flow or outbound links
            if self.network.total_o_flow[Orig] < 0.00001 or self.network.zone_outbound_link_size[Orig] == 0:
                continue

            for m in range(1, self.network.number_of_modes + 1):
                for Dest in range(1, self.network.number_of_zones + 1):
                    if Dest == Orig:
                        continue

                    RouteFlow = self.network.md_od_flow[m][Orig][Dest]
                    if RouteFlow == 0:
                        continue

                    if self.network.md_route_cost[m][Orig][Dest] >= BIG_M - 1:
                        continue

                    CurrentNode = self.network.map_external_node_id_2_node_seq_no[Dest]
                    internal_node_for_origin_node = self.network.map_external_node_id_2_node_seq_no[Orig]
                    currentLinkSequence = []

                    while CurrentNode != internal_node_for_origin_node:
                        k = MDMinPathPredLink[1][Orig][CurrentNode]

                        if k <= 0 or k > self.network.number_of_links:
                            print_log(f"A problem in All_or_Nothing_Assign() Invalid pred for node seq no {CurrentNode} Orig zone = {Orig}",2)
                            break

                        Volume[k] += RouteFlow * 1  # g_mode_type_vector[m].pce
                        ModeVolume[k][m] += RouteFlow
                        CurrentNode = self.network.links[k].internal_from_node_id

                        if CurrentNode <= 0 or CurrentNode > self.network.number_of_nodes:
                            print_log(f"A problem in All_or_Nothing_Assign() Invalid node seq no {CurrentNode} Orig zone = {Orig}",2)
                            break

                        currentLinkSequence.append(k)

                    # Store link sequence for this OD pair
                    self.add_link_sequence(m, Orig, Dest, assignment_iteration_no, currentLinkSequence)

        # Update volumes based on iteration
        if assignment_iteration_no == 0:
            for k in range(1, self.network.number_of_links + 1):
                main_volume[k] = 0
                for m in range(1, self.network.number_of_modes + 1):
                    self.network.links[k].mode_MainVolume[m] = ModeVolume[k][m]
                    main_volume[k] = main_volume[k] + ModeVolume[k][m]
                    print_log(f"AssignIterations=0: Link {k}, Mode {m}, mode_MainVolume updated to {self.network.links[k].mode_MainVolume[m]}",2)
        else:
            for k in range(1, self.network.number_of_links + 1):
                main_volume[k] = 0
                for m in range(1, self.network.number_of_modes + 1):
                    main_volume[k] = main_volume[k] + ModeVolume[k][m]
                    self.network.links[k].mode_SubVolume[m] = ModeVolume[k][m]
                    print_log(f"AssignIterations>0: Link {k}, Mode {m}, mode_SubVolume updated to {self.network.links[k].mode_SubVolume[m]}",2)

    def add_link_sequence(self, m, orig, dest, route_id, link_ids):
        """Add a link sequence for a specific mode, origin, destination, and route."""

        if not self.network.link_indices:
            return

        # Ensure we are within bounds before adding the link sequence
        if (
                0 <= m < len(self.network.link_indices) and
                0 <= orig < len(self.network.link_indices[m]) and
                0 <= dest < len(self.network.link_indices[m][orig]) and
                0 <= route_id < len(self.network.link_indices[m][orig][dest])
        ):
            self.network.link_indices[m][orig][dest][route_id] = link_ids
        else:
            print_log("Error: Invalid indices for adding link sequence.",2)

    def volume_difference(self, volume1, volume2, difference):
        for k in range(1, self.network.number_of_links + 1):
            difference[k] = volume1[k] - volume2[k]
            print_log(f"Link {k}: Volume1 = {volume1[k]}, Volume2 = {volume2[k]}, Difference = {difference[k]}", 2)
            for m in range(1, self.network.number_of_modes + 1):
                self.network.links[k].mode_MainVolume[m] = self.network.links[k].mode_SubVolume[m] - self.network.links[k].mode_MainVolume[m]

    def links_sd_line_search(self, main_volume, sd_volume):
        min_iterations = 5
        max_iterations = 5

        """
        Perform a line search using the bisection method to find the optimal step size lambda.

        Parameters:
            main_volume: Current flow volumes on each link (list or numpy array).
            sd_volume: Search direction volumes (difference between AON assignment and current flows).
            of_links_directional_derivative: Function to compute the directional derivative.
            min_iterations: Minimum iterations for the bisection method.
            max_iterations: Maximum iterations for additional convergence steps.

        Returns:
            Optimal step size (lambda).
        """
        lambdaleft = 0
        lambdaright = 1
        lambda_ = 0.5
        LastLambda = 0.5

        # Initial check at lambda = 0
        grad = self.of_links_directional_derivative(main_volume, sd_volume, 0.0)

        if grad >= 0:
            LastLambda = 0.0
            return 0.0

        # Check at lambda = 1
        grad = self.of_links_directional_derivative(main_volume, sd_volume, 1.0)
        if grad <= 0:
            LastLambda = 1.0
            return 1.0

        # Bisection method for line search within [0, 1]
        for n in range(1, min_iterations + 1):
            grad = self.of_links_directional_derivative(main_volume, sd_volume, lambda_)
            if grad < 0.0:
                lambdaleft = lambda_
            elif grad > 0.0:
                lambdaright = lambda_
            else:
                break
            lambda_ = 0.5 * (lambdaleft + lambdaright)
            LastLambda = lambda_
        # Additional iterations to ensure convergence, if necessary max_iterations is MAX_NO_BISECTITERATION
        # while lambdaleft == 0 and n <= max_iterations:
        #     grad = self.of_links_directional_derivative(main_volume, sd_volume, lambda_)
        #     if grad <= 0.0:
        #         lambdaleft = lambda_
        #     else:
        #         lambdaright = lambda_
        #     lambda_ = 0.5 * (lambdaleft + lambdaright)
        #     n += 1

        return LastLambda

    def of_links_directional_derivative(self, main_volume, sd_volume, Lambda):
        OFscale = 1
        volume = np.zeros(self.network.number_of_links + 1)
        link_cost_sum = 0

        for k in range(1, self.network.number_of_links + 1):
            volume[k] = main_volume[k] + Lambda * sd_volume[k]

        for k in range(1, self.network.number_of_links + 1):
            link_cost_sum += self.link_gen_cost(k, volume) * sd_volume[k]

        return link_cost_sum / OFscale

    def update_volume(self, main_volume, sd_volume, lambda_):
        """
        Update the main volume using the search direction and step size lambda.

        Parameters:
            main_volume: Current flow volumes on each link (list or numpy array).
            sd_volume: Search direction volumes (difference between AON assignment and current flows).
            lambda_: Step size for updating volumes.
            link_data: A dictionary or data structure holding link information, including mode_MainVolume and mode_SDVolume.
            number_of_links: Total number of links.
            number_of_modes: Total number of modes.
        """
        # Update MainVolume using Lambda * SDVolume
        for k in range(1, self.network.number_of_links + 1):
            main_volume[k] += lambda_ * sd_volume[k]
            m = 1
            self.network.links[k].mode_MainVolume[m] = main_volume[k]
            print_log(f"SDVolume = {sd_volume[k]}, Lambda = {lambda_}, Updated MainVolume = {main_volume[k]}",2)


if __name__ == "__main__":
    reader = NetworkReader()
    reader.read_data()
    solver = FlowAssignmentSolver(reader.network)
    solver.solve()