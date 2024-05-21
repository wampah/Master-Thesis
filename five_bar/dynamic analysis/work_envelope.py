import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point


df = pd.read_parquet(os.path.join(os.path.dirname(__file__), 'feasible_points.parquet'), engine='pyarrow')

custom_headers = ['q1', 'q2', 'effx', 'effy']
df.columns = custom_headers

df["theta"]=np.arctan2(df["effy"],df["effx"])
df["r"]=np.sqrt(df["effy"]**2+df["effx"]**2)

df_sorted = df.sort_values('theta')


bins = pd.cut(df_sorted['theta'], bins=100)

effx_out = []
effy_out = []

effx_in = []
effy_in = []

grouped = df_sorted.groupby(bins)
for _, group in grouped:
    if not group.empty:
        min_row = group.loc[group['r'].idxmin()]
        max_row = group.loc[group['r'].idxmax()]
        effx_in.append(min_row['effx'])
        effy_in.append(min_row['effy'])
        effx_out.append(max_row['effx'])
        effy_out.append(max_row['effy'])

fig1,ax1=plt.subplots()

ax1.plot(effx_out,effy_out,"r*")
ax1.plot(effx_in,effy_in,"k*")


eff_vals=df[["effx","effy"]].to_numpy()
ax1.plot(eff_vals[:,0], eff_vals[:,1], '*',markersize=0.5)

plt.show()
def generate_random_points_between_polygons(outer_polygon, inner_polygon, num_points):
    pointsx = []
    pointsy = []
    minx, miny, maxx, maxy = outer_polygon.bounds
    while len(pointsx) < num_points:
        random_point = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
        if outer_polygon.contains(random_point) and not inner_polygon.contains(random_point):
            pointsx.append(random_point.x)
            pointsy.append(random_point.y)
    return pointsx,pointsy


outer_polygon = Polygon(list(zip(effx_out, effy_out)))
inner_polygon = Polygon(list(zip(effx_in, effy_in)))


random_pointsx,random_pointsy = generate_random_points_between_polygons(outer_polygon, inner_polygon, 1e5)

a=np.asarray([random_pointsx,random_pointsy])
np.savetxt("foo.csv", a, delimiter=",")

fig2,ax2=plt.subplots()


ax2.plot(random_pointsx,random_pointsy,"g*")


plt.show()