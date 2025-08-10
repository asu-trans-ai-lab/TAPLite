# -*- coding: utf-8 -*-
"""
Created on Fri Jan 24 21:28:46 2025

@author: xzhou
"""

import pandas as pd
import numpy as np
from shapely.geometry import Point, LineString

# Load the planning activity nodes and OSM activity nodes
planning_nodes_path = 'planning_activity_node.csv'  # Replace with the correct path
osm_nodes_path = 'osm_activity_nodes.csv'  # Replace with the correct path

planning_nodes = pd.read_csv(planning_nodes_path)
osm_nodes = pd.read_csv(osm_nodes_path)

# Extract necessary columns (node_id, geometry)
planning_nodes = planning_nodes[["node_id", "geometry"]]
osm_nodes = osm_nodes[["node_id", "geometry"]]

# Convert geometry strings to Shapely Points
planning_nodes['geometry'] = planning_nodes['geometry'].apply(lambda x: Point(eval(x.replace("POINT ", ""))))
osm_nodes['geometry'] = osm_nodes['geometry'].apply(lambda x: Point(eval(x.replace("POINT ", ""))))

# Compute the pairwise shortest distances between planning and OSM activity nodes
distances = []
for _, plan_row in planning_nodes.iterrows():
    plan_point = plan_row['geometry']
    min_distance = float('inf')
    closest_osm = None
    for _, osm_row in osm_nodes.iterrows():
        osm_point = osm_row['geometry']
        distance = plan_point.distance(osm_point)
        if distance < min_distance:
            min_distance = distance
            closest_osm = osm_row['node_id']
    distances.append((plan_row['node_id'], closest_osm, min_distance))

# Create a DataFrame to store the results
distances_df = pd.DataFrame(distances, columns=["planning_node_id", "osm_node_id", "distance"])

# Sort by the largest distance
distances_df = distances_df.sort_values(by="distance", ascending=False)

# Compute the average distance
average_distance = distances_df['distance'].mean()
print(f"Average distance: {average_distance:.2f}")

# Filter the top 100 longest distances
top_100_distances = distances_df.head(100)

# Generate WKT lines for the top 100 longest distances
top_100_distances['wkt'] = top_100_distances.apply(
    lambda row: LineString([
        planning_nodes.loc[planning_nodes['node_id'] == row['planning_node_id'], 'geometry'].values[0],
        osm_nodes.loc[osm_nodes['node_id'] == row['osm_node_id'], 'geometry'].values[0]
    ]).wkt, axis=1)

# Save the top 100 distances to a CSV file
top_100_output_path = 'top_100_longest_distances.csv'
top_100_distances.to_csv(top_100_output_path, index=False)

print(f"Top 100 longest distances saved to: {top_100_output_path}")

# Save the full matching file with WKT to a CSV
matching_output_path = 'matching_distances_wkt.csv'
distances_df['wkt'] = distances_df.apply(
    lambda row: LineString([
        planning_nodes.loc[planning_nodes['node_id'] == row['planning_node_id'], 'geometry'].values[0],
        osm_nodes.loc[osm_nodes['node_id'] == row['osm_node_id'], 'geometry'].values[0]
    ]).wkt, axis=1)
distances_df.to_csv(matching_output_path, index=False)

print(f"Full matching file with WKT saved to: {matching_output_path}")
