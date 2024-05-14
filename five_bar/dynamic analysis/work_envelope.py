import pandas as pd
import os
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d

# Load the CSV into a DataFrame
df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'feasible_points.csv'), delimiter=",", header=None, names=['q1', 'q2', 'effx', 'effy'])

# Sort the DataFrame by 'effx'
df_sorted = df.sort_values('effx')

# Create 1000 bins for 'effx'
bins = pd.cut(df_sorted['effx'], bins=50)

# Initialize lists to store the results
min_effx = []
min_effy = []
max_effx = []
max_effy = []

# Group by the bins and find the min and max 'effy' and their associated 'effx'
grouped = df_sorted.groupby(bins)
for _, group in grouped:
    if not group.empty:
        min_row = group.loc[group['effy'].idxmin()]
        max_row = group.loc[group['effy'].idxmax()]
        min_effx.append(min_row['effx'])
        min_effy.append(min_row['effy'])
        max_effx.append(max_row['effx'])
        max_effy.append(max_row['effy'])

# Create a new DataFrame with the results
result = pd.DataFrame({
    'min_effx': min_effx,
    'min_effy': min_effy,
    'max_effx': max_effx,
    'max_effy': max_effy
})

min_effx_vals=result["min_effx"].to_numpy()
min_effy_vals=result["min_effy"].to_numpy()
max_effx_vals=result["max_effx"].to_numpy()
max_effy_vals=result["max_effy"].to_numpy()

plt.plot(min_effx_vals,min_effy_vals,"ro")
plt.plot(max_effx_vals,max_effy_vals,"ro")




eff_vals=df[["effx","effy"]].to_numpy()

hull = ConvexHull(eff_vals)
plt.plot(eff_vals[:,0], eff_vals[:,1], '*',markersize=1)

for simplex in hull.simplices:
    plt.plot(eff_vals[simplex, 0], eff_vals[simplex, 1], 'k-')
plt.show()