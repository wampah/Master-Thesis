import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.strtree import STRtree

# Read Data
df = pd.read_parquet(os.path.join(os.path.dirname(__file__), 'feasible_points.parquet'), engine="pyarrow")

# Rename columns
df.columns = ['q1', 'q2', 'effx', 'effy']

# Convert to polar
df["theta"] = np.arctan2(df["effy"], df["effx"])
df["r"] = np.sqrt(df["effy"]**2 + df["effx"]**2)

# Sort by theta
df_sorted = df.sort_values('theta')

# Compute outer and inner boundaries
bins = pd.cut(df_sorted['theta'], bins=100)
effx_out, effy_out, effx_in, effy_in = [], [], [], []

for _, group in df_sorted.groupby(bins):
    if not group.empty:
        min_row, max_row = group.loc[group['r'].idxmin()], group.loc[group['r'].idxmax()]
        effx_in.append(min_row['effx']), effy_in.append(min_row['effy'])
        effx_out.append(max_row['effx']), effy_out.append(max_row['effy'])

# Define Polygons
outer_polygon = Polygon(list(zip(effx_out, effy_out)))
inner_polygon = Polygon(list(zip(effx_in, effy_in)))

# Faster Random Sampling
def generate_random_points_fast(outer_polygon, inner_polygon, num_points):
    minx, miny, maxx, maxy = outer_polygon.bounds
    random_x = np.random.uniform(minx, maxx, num_points)
    random_y = np.random.uniform(miny, maxy, num_points)
    points = np.vstack((random_x, random_y)).T
    point_objs = [Point(p) for p in points]

    # Use STRtree for fast containment check
    tree = STRtree(point_objs)
    inside_outer = np.array([outer_polygon.contains(p) for p in point_objs])
    outside_inner = np.array([not inner_polygon.contains(p) for p in point_objs])

    valid_indices = inside_outer & outside_inner
    return points[valid_indices, 0], points[valid_indices, 1]

# Generate Faster
random_pointsx, random_pointsy = generate_random_points_fast(outer_polygon, inner_polygon, int(1e6))
print("Generated x Points:",len(random_pointsx))
print("Generated y Points:",len(random_pointsy))
# Save Faster
pd.DataFrame({"x": random_pointsx, "y": random_pointsy}).to_csv(os.path.join(os.path.dirname(__file__), 'random_pts.csv'), index=False)

# First Plot: Original Feasible Points and Boundaries
fig1, ax1 = plt.subplots()
ax1.scatter(df["effx"], df["effy"], s=0.01, color="b", label="Feasible Points")
ax1.scatter(effx_out, effy_out, s=10, color="r", label="Outer Boundary", marker="*")
ax1.scatter(effx_in, effy_in, s=10, color="k", label="Inner Boundary", marker="*")
ax1.legend()
ax1.set_title("Feasible Points with Inner/Outer Boundaries")

# Second Plot: Random Sampled Points
fig2, ax2 = plt.subplots()
ax2.scatter(random_pointsx, random_pointsy, s=0.01, color="k", label="Random Points")
ax2.legend()
ax2.set_title("Random Points Inside Feasible Region")

plt.show()