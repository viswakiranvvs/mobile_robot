import matplotlib.pyplot as plt
import numpy as np
import json
import os
from sklearn.metrics import root_mean_squared_error, r2_score

mesh_folder = os.path.join(os.getcwd(), "meshes")
json_path = os.path.join(mesh_folder, "dist_degree.json")
degree_dist_dict = {}
with open(json_path, "r") as f:
    degree_dist_dict=json.load(f)

fig, ax = plt.subplots(figsize=(10, 6))
degrees = list(degree_dist_dict.keys())
distances = list(degree_dist_dict.values())
reduced_degrees = []
reduced_distances = []
original_distance = []
for i in range(len(degrees)):
    if i%5 == 0:
        reduced_degrees.append(degrees[i])
        reduced_distances.append(distances[i])
        original_distance.append(2.89)

# Calculate RMSE and R²
rmse = root_mean_squared_error(original_distance, reduced_distances)
r2 = r2_score(original_distance, reduced_distances)
print(f"RMSE: {rmse:.4f}")
print(f"R²: {r2:.4f}")
high_deviation_threshold = 0.3
dev_counts = 0
for i in range(len(reduced_distances)):
    if abs(reduced_distances[i] - original_distance[i]) > high_deviation_threshold:
        dev_counts += 1
accuracy = (1 - (dev_counts / len(reduced_distances))) * 100
print(f"Accuracy: {accuracy:.2f}%")
# print(f"Highly deviated points")
ax.plot(reduced_degrees, reduced_distances, marker='o', linestyle='-', label='Estimated Distance')
# ax.plot(degrees, distances, alpha=0.3)
ax.plot(reduced_degrees, original_distance, linestyle='--', color='r', label='Original Distance (2.89m)')
ax.set_xlabel('Degree')
ax.set_ylabel('Distance (m)')
ax.set_title('LIDAR Distance vs Degree')
ax.grid(True)
ax.legend()
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig(os.path.join(mesh_folder, "lidar_distance_vs_degree.png"))
plt.show()