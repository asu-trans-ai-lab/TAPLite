# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 16:43:15 2024

@author: xzhou
"""

import csv
import os
from collections import defaultdict

import numpy as np


MAX_MODE_TYPES = 10
# Global Variables
number_of_zones = 0
number_of_modes = 1
number_of_nodes = 0
number_of_links = 0
first_thru_node = 0

AssignIterations = 20
demand_period_starting_hours = 7
demand_period_ending_hours = 8
g_tap_log_file = 0
g_base_demand_mode = 1
g_ODME_mode = 0
g_ODME_obs_VMT = -1
g_System_VMT = 0

g_ODME_link_volume_penalty = 0.01  # Relative weight on volume, converts deviation of link volume to travel time
g_ODME_VMT_penalty = 0.01

# Global dictionaries
g_map_external_node_id_2_node_seq_no = {}  # Maps external node ID to node sequence number
g_map_node_seq_no_2_external_node_id = {}  # Maps node sequence number to external node ID

Link = []  # A list to hold LinkRecord objects

# Replace `int* FirstLinkFrom;` and `int* LastLinkFrom;` with Python lists
FirstLinkFrom = []  # A list to store the first link from each node
LastLinkFrom = []   # A list to store the last link from each node

# Replace `sorted_list* LinksTo;` with a dictionary of lists for better handling
LinksTo = {}  # A dictionary where keys are node IDs and values are lists of links directed to the node


# Placeholder definitions

total_o_flow = None       # Placeholder for a 1D array
zone_outbound_link_size = None  # Placeholder for a 1D integer array
md_od_flow = None         # Placeholder for a 3D array
md_route_cost = None      # Placeholder for a 3D array


   
class ModeType:
    """Represents a mode type with its associated attributes."""
    def __init__(self, mode_type, vot, pce, occ, dedicated_shortest_path, demand_file):
        self.mode_type = mode_type  # Mode type as a string
        self.vot = vot  # Value of time
        self.pce = pce  # Passenger car equivalent
        self.occ = occ  # Occupancy
        self.dedicated_shortest_path = dedicated_shortest_path  # 1 if dedicated, 0 otherwise
        self.demand_file = demand_file  # Path to the demand file


# Equivalent to the array g_mode_type_vector[MAX_MODE_TYPES]

g_mode_type_vector = [None] * MAX_MODE_TYPES  # Create a list for mode types

class LinkRecord:
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

        self.Travel_time = 0.0
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
        self.VDF_Beta = 4
        self.VDF_plf = 1.0
        self.Q_cd = 1.0
        self.Q_n = 1.0
        self.Q_cp = 0.28125
        self.Q_s = 4.0
        self.Travel_time = 0.0
        self.BPR_TT = 0.0
        self.QVDF_TT = 0.0
        self.Ref_volume = 0.0
        self.Base_volume = 0.0
        self.Obs_volume = -1.0

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

def process_node_file(file_name, log_file=None):
    """
    Reads node.csv and processes its data.
    
    :param file_name: Path to the node.csv file.
    :param log_file: Path to the log file, if any.
    :return: A tuple with the number of nodes, number of zones, 
             first through node, and mappings.
    """
    # Initialize variables
    global g_map_external_node_id_2_node_seq_no, g_map_external_node_id_2_node_seq_no, number_of_zones, number_of_nodes, first_thru_node


    l_first_thru_node = first_thru_node
    tap_log_enabled = log_file is not None

    # Open the log file if provided
    logfile = open(log_file, 'w') if log_file else None

    # Open and read the CSV file
    with open(file_name, 'r') as csv_file:
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
            g_map_node_seq_no_2_external_node_id[number_of_nodes + 1] = node_id
            g_map_external_node_id_2_node_seq_no[node_id] = number_of_nodes + 1  # Sequential number starts from 1

            # Update number_of_zones
            if zone_id >= 1 and zone_id > number_of_zones:
                number_of_zones = zone_id

            # Set first through node if not initialized
            if zone_id == 0 and l_first_thru_node == first_thru_node:  # Not initialized
                l_first_thru_node = number_of_nodes + 1

            # Log data if tap_log_file is enabled
            if tap_log_enabled:
                logfile.write(f"node_id = {node_id}, node_seq_no = {g_map_external_node_id_2_node_seq_no[node_id]}\n")

            # Increment node count
            number_of_nodes += 1

    # Close the log file if it was opened
    if logfile:
        logfile.close()

    print(g_map_node_seq_no_2_external_node_id)
            
    return




def get_number_of_links_from_link_file(file_name):
    """
    Reads a CSV file and counts the number of links based on its records.
    
    :param file_name: Path to the link.csv file.
    :return: The total number of links in the file.
    """
    global number_of_links
    number_of_links = 0

    # Open and read the CSV file
    with open(file_name, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        
        for row in csv_reader:
            # Read link_id and from_node_id (values are not used in counting)
            link_id = int(row.get("link_id", 0))
            from_node_id = int(row.get("from_node_id", 0))
            
            # Increment the link count
            number_of_links += 1

    return number_of_links


def read_links(file_name, number_of_modes):
    global number_of_links, g_map_external_node_id_2_node_seq_no, links
    g_tap_log_file=0
    logfile_path=None
    links = defaultdict(LinkRecord)  # Dictionary to store links with their index as keys
    total_base_link_volume = 0
    k = 1  # Link index starts from 1

    with open(file_name, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)

        for row in csv_reader:
            link = LinkRecord()
            link.setup(number_of_modes)

            # Read basic properties
            link.external_from_node_id = int(row["from_node_id"])
            link.external_to_node_id = int(row["to_node_id"])
            link.link_id = int(row["link_id"])
            link.link_type = int(row["link_type"])

            # Map internal node IDs
            link.internal_from_node_id = g_map_external_node_id_2_node_seq_no.get(link.external_from_node_id, 0)
            if link.internal_from_node_id == 0:
                print(f"Error in from_node_id = {link.external_from_node_id} for link_id = {link.link_id}")
                continue

            link.internal_to_node_id = g_map_external_node_id_2_node_seq_no.get(link.external_to_node_id, 0)
            if link.internal_to_node_id == 0:
                print(f"Error in to_node_id = {link.external_to_node_id} for link_id = {link.link_id}")
                continue

            # Read additional properties
            link.length = float(row.get("length", 0.0))
            
            link.Ref_volume = float(row.get("ref_volume", 0.0))


            if "lanes" in row:
                link.lanes = int(row["lanes"])
            if "capacity" in row:
                link.Lane_Capacity = float(row["capacity"])
                link.Link_Capacity = link.lanes * link.Lane_Capacity

            link.free_speed = float(row.get("free_speed", 10.0))
            link.FreeTravelTime = link.length / link.free_speed * 60.0

            debug = 1
             # Debugging output
            if debug and k <=3:
                print("Processing link properties:")
                print(f"  Length: {link.length }")
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

            links[k] = link
            k += 1

    print(f"total_base_link_volume = {total_base_link_volume}")
    baselinkvolume_loaded_flag = 1 if total_base_link_volume > 0 else 0

    return links

def find_links_to():
    """
    Finds the sorted list of links directed to each node.

    :param no_nodes: Number of nodes.
    :param number_of_links: Number of links.
    :param links: List of Link objects indexed by 1-based index.
    :return: A dictionary where keys are node IDs, and values are sorted lists of link IDs.
    """
    global links_to, links, number_of_links, number_of_nodes
    # Initialize a dictionary with empty lists for each node
    links_to = {node: [] for node in range(1, number_of_nodes + 1)}

    # Populate the links_to dictionary
    for k in range(1, number_of_links + 1):
        internal_to_node_id = links[k].internal_to_node_id
        links_to[internal_to_node_id].append(k)

    # Sort the lists for each node
    for node in links_to:
        links_to[node].sort()

    return links_to

def init_link_pointers(links_file_name,  g_tap_log_file=0, logfile_path=None):
    """
    Initializes pointers to the first and last link originating from each node.

    :param links_file_name: Name of the links file (for warnings and messages).
    :param no_nodes: Total number of nodes.
    :param number_of_links: Total number of links.
    :param links: List of Link objects indexed by 1-based index.
    :param g_map_node_seq_no_2_external_node_id: Map of internal to external node IDs.
    :param g_tap_log_file: Flag for logging output (1 = enabled, 0 = disabled).
    :param logfile_path: Path to the log file (if logging is enabled).
    :return: Tuple of FirstLinkFrom and LastLinkFrom lists.
    """
    # Initialize FirstLinkFrom and LastLinkFrom lists
    global FirstLinkFrom, LastLinkFrom , number_of_nodes, number_of_links
    FirstLinkFrom = [0] * (number_of_nodes + 1)  # 1-based indexing
    LastLinkFrom = [-1] * (number_of_nodes + 1)  # -1 indicates no links

    FirstLinkFrom[1] = 1
    Node = 1

    for k in range(1, number_of_links + 1):
        internal_from_node_id = links[k].internal_from_node_id

        if internal_from_node_id == Node:
            continue
        elif internal_from_node_id >= Node + 1:
            LastLinkFrom[Node] = k - 1
            Node = internal_from_node_id
            FirstLinkFrom[Node] = k
        elif internal_from_node_id < Node:
            raise ValueError(f"Sort error in link file '{links_file_name}': "
                             f"a link from node {internal_from_node_id} was found after a link from node {Node}.")
        elif internal_from_node_id > Node + 1:
            # Handle nodes with no links
            LastLinkFrom[Node] = k - 1
            for Node in range(Node + 1, internal_from_node_id):
                FirstLinkFrom[Node] = 0
                LastLinkFrom[Node] = -1
            FirstLinkFrom[Node] = k

    if Node == number_of_nodes:
        LastLinkFrom[Node] = number_of_links
    else:
        # Handle remaining nodes with no links
        LastLinkFrom[Node] = number_of_links - 1
        for Node in range(Node + 1, no_nodes + 1):
            FirstLinkFrom[Node] = 0
            LastLinkFrom[Node] = -1

    # Optional logging
    if g_tap_log_file == 1 and logfile_path:
        with open(logfile_path, 'w') as logfile:
            for Node in range(1, number_of_nodes + 1):
                logfile.write(f"node_id = {g_map_node_seq_no_2_external_node_id.get(Node, Node)}, "
                              f"FirstLinkFrom = {FirstLinkFrom[Node]}, "
                              f"LastLinkFrom = {LastLinkFrom[Node]} \n")

    return




def init_links(links_file_name="link.csv", g_tap_log_file=0, logfile_path=None):
    """
    Initializes link data by reading the links file, setting up the 'LinksTo' structure, and initializing link pointers.

    :param links_file_name: Name of the links file.

 
    :param g_tap_log_file: Flag for logging output (1 = enabled, 0 = disabled).
    :param logfile_path: Path to the log file (if logging is enabled).
    :return: Tuple containing links, LinksTo, FirstLinkFrom, and LastLinkFrom.
    """
    global links, links_to, FirstLinkFrom, LastLinkFrom, number_of_links, number_of_nodes, g_map_node_seq_no_2_external_node_id
    # Initialize the links structure
    links = [None] * (number_of_links + 1)  # +1 for 1-based indexing

    # Step 1: Read links from file
    links = read_links(links_file_name, number_of_modes=10)

    # Step 2: Find 'LinksTo' structure
    links_to = find_links_to()

    # Step 3: Initialize link pointers
    init_link_pointers(links_file_name, g_tap_log_file, logfile_path)

    return



def sum_od_table(od_table, total_o_table, no_zones):
    """
    Calculate the sum of the OD table and populate total_o_table.
    """
    total_o_table.fill(0)
    total_sum = 0.0

    for m in range(od_table.shape[0]):
        for orig in range(1, no_zones + 1):
            for dest in range(1, no_zones + 1):
                total_o_table[orig] += od_table[m, orig, dest]
                total_sum += od_table[m, orig, dest]

    # Print the OD sum volume for each zone
    print("OD Sum Volume for Each Zone:")
    for orig in range(1, no_zones + 1):
        print(f"Zone {orig}: {total_o_table[orig]:.4f}")\
            
    # Print the total OD volume
    print(f"Total OD Volume Across All Zones: {total_sum:.4f}")
    
    return total_sum

def read_od_flow():
    """
    Read the OD flow and initialize required data structures.
    """
    global number_of_modes, number_of_zones, links,  md_od_flow, md_route_cost, total_o_flow
    od_table_shape = (number_of_modes+1, number_of_zones + 1, number_of_zones + 1)
    md_od_flow = alloc_3d(od_table_shape)
    total_o_flow = alloc_1d(number_of_zones + 1)

    md_route_cost = alloc_3d(od_table_shape)

    # Populate MDODflow
    read_od_table(md_od_flow)

    # Calculate the total OD flow
    real_total = sum_od_table(md_od_flow, total_o_flow, number_of_zones)

    # Calculate outbound link size
    zone_outbound_link_size = alloc_1d(number_of_zones + 1, default_value=0)
    for k in range(1, number_of_links + 1):
        if  links[k].external_from_node_id <= number_of_zones:
            zone_outbound_link_size[links[k].external_from_node_id ] += 1

    # Error checking
    for z in range(1, number_of_zones + 1):
        if zone_outbound_link_size[z] == 0 and total_o_flow[z] > 0.01:
            raise ValueError(f"No outbound link from zone {z} with positive demand {total_o_flow[z]}")

    return

def read_od_table(od_table):
    """
    Read OD table and populate OD and DiffOD tables.
    """
    global number_of_zones
    
    m = 0 
    demand_file = 'demand.csv'
    print(f"Reading demand file: {demand_file}")

    line_count = 0

    try:
        with open(demand_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                o_zone_id = int(row["o_zone_id"])
                d_zone_id = int(row["d_zone_id"])
                volume = float(row["volume"])

                if o_zone_id > number_of_zones or d_zone_id > number_of_zones:
                    raise ValueError(f"Invalid zone ID in {demand_file}: {o_zone_id}, {d_zone_id}")

                if line_count <= 3:
                    print(f"o_zone_id: {o_zone_id}, d_zone_id: {d_zone_id}, volume: {volume:.4f}")
                    line_count = line_count + 1


                od_table[m, o_zone_id, d_zone_id] = volume

    except FileNotFoundError:
        print(f"Error: File {demand_file} not found.")
        return
# Constants
INVALID = -1
BIGM = 9999999
WAS_IN_QUEUE = -7

def minpath(mode, orig, pred_link, cost_to):
    """
    Computes the shortest path from an origin node using a modified label-setting algorithm.
    """
    global FirstLinkFrom, LastLinkFrom, g_map_external_node_id_2_node_seq_no, g_map_external_node_id_2_node_seq_no, number_of_zones, number_of_nodes, first_thru_node
    cost_to.fill(BIGM)
    pred_link.fill(INVALID)
    
    # Initialize variables
    queue_next = alloc_1d_int(number_of_nodes+1)
    
    now = g_map_external_node_id_2_node_seq_no[orig]
    internal_node_id_for_origin_zone = now
   
    pred_link[now] = INVALID
    cost_to[now] = 0.0

    scan_list = []
    scan_list.append(now)
    return2q_count = 0
    
    debug  = 1 

    
    if debug:
        print(f"Starting minpath computation for mode {mode}, origin {orig}")
        print(f"Initial node: {now}, queue initialized")

    while scan_list:
        now = scan_list.pop(0)  # Remove the first element from the scan list
        if now >= first_thru_node or now == internal_node_id_for_origin_zone:
            if debug:
                print(f"Processing node {now}...")
                
            for k in range(FirstLinkFrom[now], LastLinkFrom[now] + 1):
            #    if links[k]["mode_allowed_use"][mode] == 0:
            #        continue

                new_node = links[k].internal_to_node_id
                new_cost = cost_to[now] + links[k].length 

                if debug:
                    print(f"Checking link {k}: new_node={new_node}, links[k].length  = {links[k].length}, new_cost={new_cost:.4f}, "
                          f"current_cost={cost_to[new_node]:.4f}")

                if cost_to[new_node] > new_cost:
                    
                    if debug:
                        print(f"Updated cost for node {new_node}: {new_cost:.4f}")
                        print(f"Predecessor for node {new_node}: link {k}")
    
                    cost_to[new_node] = new_cost
                    pred_link[new_node] = k

 # Add the node to the scan list if it's not already there
                    if new_node not in scan_list:
                        scan_list.append(new_node)
                        if debug:
                            print(f"    Node {new_node} added to scan list")

    if debug:
        print("Updated cost_to array:")
        for idx, cost in enumerate(cost_to):
            print(f"  Node {idx}: Cost = {cost:.4f}")
    
        print("Updated pred_link array:")
        for idx, pred in enumerate(pred_link):
            print(f"  Node {idx}: Predecessor Link = {pred}")
    
    if debug:
        print("Finished minpath computation")
        print(f"Total nodes returned to queue: {return2q_count}")
            
        return return2q_count

def find_min_cost_routes(min_path_pred_link):
    """
    Finds the minimum cost routes for all origins and modes.
    """
    global g_map_external_node_id_2_node_seq_no, total_o_flow, md_route_cost, md_od_flow, zone_outbound_link_size, number_of_zones, number_of_nodes
    cost_to = np.full((number_of_zones + 1, number_of_nodes + 1), BIGM, dtype=float)

    system_least_travel_time  = 0 
    m = 0 
    debug = 1
    for orig in range(1, number_of_zones):
        if total_o_flow[orig] < 1e-5:
            continue
    

        
        minpath(m, orig, min_path_pred_link[m][orig], cost_to[orig])

        if md_route_cost is not None:
            for dest in range(1, number_of_zones + 1):
                md_route_cost[m][orig][dest] = BIGM

                if md_od_flow[m][orig][dest] > 1e-6:
                    internal_node_id_for_destination_zone = g_map_external_node_id_2_node_seq_no[dest]
                    if cost_to[orig][internal_node_id_for_destination_zone] <= BIGM - 1:
                        md_route_cost[m][orig][dest] = cost_to[orig][internal_node_id_for_destination_zone]
                        system_least_travel_time+= (
                            md_route_cost[m][orig][dest] * md_od_flow[m][orig][dest] 
                        )

                    if debug:
                       print(f"Processing mode {m}, origin {orig}") 
                       print(f"    Destination {dest}:")
                       print(f"      Cost to Destination: {cost_to[orig][internal_node_id_for_destination_zone]:.4f}")
                       print(f"      Updated md_route_cost[{m}][{orig}][{dest}] = {md_route_cost[m][orig][dest]:.4f}")
                       print(f"      Updated System Least Travel Time: {system_least_travel_time:.4f}")
   
        return system_least_travel_time
  
# main program below   


# Example Usage
file_name = "node.csv"  # Replace with your CSV file path
log_file = "logfile.txt"  # Optional log file path

# Process the node file
process_node_file(file_name, log_file=log_file)



# Example Usage
file_name = "link.csv"  # Replace with your CSV file path

# Get the number of links
number_of_links = get_number_of_links_from_link_file(file_name)

# Output the result
print(f"Number of Links: {number_of_links}")


# Output results
print(f"Number of Nodes: {number_of_nodes}")
print(f"Number of Zones: {number_of_zones}")
print(f"First Through Node: {first_thru_node}")
#print(f"Node Sequence Map: {g_map_node_seq_no_2_external_node_id}")
#print(f"External Node Map: {external_node_map}")


# Open the summary log file for writing
summary_log_file = open("summary_log_file.txt", "w")

# Initialize variables
MainVolume = None  # Placeholder for a double array
SubVolume = None  # Placeholder for a double array
SDVolume = None  # Placeholder for a double array
Lambda = 0.0
MDMinPathPredLink = None  # Placeholder for a 3D array

init_links(
    links_file_name="link.csv",
    g_tap_log_file=1,
    logfile_path="logfile.txt"
)


# Print results
print("Links:", links)
print("LinksTo:", links_to)
print("FirstLinkFrom:", FirstLinkFrom)
print("LastLinkFrom:", LastLinkFrom)


read_od_flow() 

# Allocate MDMinPathPredLink
MDMinPathPredLink = alloc_3d_int((number_of_modes + 1, number_of_links + 1, number_of_nodes + 1))

# Calculate the system least travel time
system_least_travel_time = find_min_cost_routes(MDMinPathPredLink)

# Output
print(f"System Least Travel Time: {system_least_travel_time:.2f}")
 