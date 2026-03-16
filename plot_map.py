# import json
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Load JSON file
# with open("pathTaken.json", "r") as f:
#     data = json.load(f)

# x_vals = []
# y_vals = []
# z_vals = []

# # Extract position values
# for obj in data:
#     x_vals.append(obj["position"]["x"])
#     y_vals.append(obj["position"]["y"])
#     z_vals.append(obj["position"]["z"])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Plot trajectory line
# ax.plot(x_vals, y_vals, z_vals)

# # Add direction arrows
# for i in range(len(x_vals) - 1):
#     dx = x_vals[i+1] - x_vals[i]
#     dy = y_vals[i+1] - y_vals[i]
#     dz = z_vals[i+1] - z_vals[i]

#     ax.quiver(
#         x_vals[i], y_vals[i], z_vals[i],
#         dx, dy, dz,
#         length=0.02,
#         normalize=True
#     )

# # Mark start node
# ax.scatter(x_vals[0], y_vals[0], z_vals[0], s=80)
# ax.text(
#     x_vals[0], y_vals[0], z_vals[0],
#     "Drone Start Position"
# )

# # Mark end node
# ax.scatter(x_vals[-1], y_vals[-1], z_vals[-1], s=80)
# ax.text(
#     x_vals[-1], y_vals[-1], z_vals[-1],
#     "Object Node"
# )

# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# plt.title("3D Drone Trajectory")

# plt.show()





import json
import plotly.graph_objects as go

with open("pathTaken.json") as f:
    data = json.load(f)

x = [d["position"]["x"] for d in data]
y = [d["position"]["y"] for d in data]
z = [d["position"]["z"] for d in data]

fig = go.Figure()

fig.add_trace(go.Scatter3d(
    x=x,
    y=y,
    z=z,
    mode="lines+markers",
    name="Trajectory"
))

fig.add_trace(go.Scatter3d(
    x=[x[0]],
    y=[y[0]],
    z=[z[0]],
    mode="markers+text",
    text=["Initial Drone Position"],
    textposition="bottom center",
    textfont=dict(size=20),
    marker=dict(size=12)
))

fig.add_trace(go.Scatter3d(
    x=[x[-1]],
    y=[y[-1]],
    z=[z[-1]],
    mode="markers+text",
    text=["Object Position"],
    textfont=dict(size=20),
    textposition="middle right",
    marker=dict(size=12)
))

fig.update_layout(
    title="Drone Trajectory",
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z"
    )
)

fig.show()