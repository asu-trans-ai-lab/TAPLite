# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 16:43:15 2024


"""
# =============================================================================
# Introduction to GMNS in the Four-Step Process and Simple Simulation
# The General Modeling Network Specification (GMNS) provides a standardized, open-source framework for representing multimodal transportation networks. Designed to streamline the process of network modeling, GMNS allows researchers, planners, and engineers to utilize consistent data structures across various tools and methodologies. This learning framework focuses on leveraging GMNS for implementing the classical four-step transportation planning process and simple simulation tasks.
# 
# The Four-Step Process with GMNS
# Trip Generation: GMNS enables the integration of land-use and socio-economic data to define zone-based productions and attractions. By parsing activity nodes and linking them to specific zones, GMNS helps to estimate trip generation using standardized production and attraction models.
# 
# Trip Distribution: The GMNS network structure supports the application of gravity models and destination choice models, such as the logit-based approach, for distributing trips between zones. These models rely on accessibility metrics derived from network data to compute flow volumes between origin-destination pairs.
# 
# Mode Choice: GMNS accommodates multimodal networks, allowing for the integration of mode-specific parameters such as travel time, capacity, and mode-specific costs. This enables the estimation of mode splits based on generalized costs for various transport modes.
# 
# Route Assignment: The robust link and node attributes in GMNS facilitate route assignment procedures like All-or-Nothing (AoN) and user equilibrium assignments. These procedures rely on the accurate representation of network link costs and capacity constraints provided by GMNS.
# 
# Simple Simulation with GMNS
# GMNS simplifies the process of conducting basic network simulations by providing:
# 
# Pre-processed Network Attributes: Default attributes such as lane capacity, free-flow speed, and link lengths are included, reducing the effort needed for network setup.
# Accessibility Analysis: Tools to compute accessibility matrices for origin-destination pairs, essential for understanding network connectivity and travel costs.
# Demand-Supply Integration: The network structure seamlessly integrates demand inputs (trip volumes) with supply-side characteristics (capacity, speed, etc.) for simulation tasks.
# Performance Evaluation: Outputs such as travel times, volumes, and congestion indices enable detailed evaluation of network performance.
# By adopting GMNS, users can efficiently transition from raw network data to actionable insights, ensuring consistency, interoperability, and scalability in transportation planning and simulation.
# =============================================================================

import csv
import os
import osm2gmns as og




def osm2gmns_network():
    
    input_file = 'test_01.osm'
    net = og.getNetFromFile(input_file) 
    
    og.consolidateComplexIntersections(net, auto_identify=True, int_buffer= 1000)
    og.fillLinkAttributesWithDefaultValues(net, default_lanes=True, default_speed=True, default_capacity=True)
    og.generateNodeActivityInfo(net)
    og.outputNetToCSV(net)


# main program below   
osm2gmns_network()
