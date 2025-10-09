#!/usr/bin/env python3

# Debug what pandas sees
import pandas as pd


path = "/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/scripts/"

file = "alum_corr_b_scans2.csv"

# Try reading without assuming column structure
raw_data = pd.read_csv(f'{path}{file}', sep=',', header=None)
print("Shape:", raw_data.shape)
print("Columns:", raw_data.columns.tolist())
print("First few rows:")
print(raw_data.head())

# # Or try reading as plain text first
# with open(filepath, 'r') as f:
#     first_line = f.readline()
#     print("First line:", repr(first_line))