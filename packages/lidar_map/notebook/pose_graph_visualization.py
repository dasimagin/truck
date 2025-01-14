import json
import os

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np


def draw_pose_graph(json_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    with open(json_path, "r") as file:
        data = json.load(file)

    image_files = []

    for key in data:
        last_iteration = key
        pose_graph_info = data[last_iteration]

        vertices = pose_graph_info["vertices"]
        edges = pose_graph_info["edges"]

        plt.figure(figsize=(15, 15))

        for vertex in vertices:
            x = vertex["x"]
            y = vertex["y"]
            theta = vertex["theta"]
            plt.plot(x, y, "ko")
            arrow_length = 2
            plt.arrow(
                x,
                y,
                arrow_length * np.cos(theta),
                arrow_length * np.sin(theta),
                head_width=0.05,
                head_length=0.1,
                fc="black",
                ec="black",
            )

        for edge in edges:
            from_vertex = next(v for v in vertices if v["id"] == edge["from_edge"])
            to_vertex = next(v for v in vertices if v["id"] == edge["to_edge"])

            x_from = from_vertex["x"]
            y_from = from_vertex["y"]
            x_to = to_vertex["x"]
            y_to = to_vertex["y"]

            color = "red" if edge["type"] == "odometry" else "green"
            linewidth = 4 if edge["type"] == "icp" else 1
            plt.plot([x_from, x_to], [y_from, y_to], color=color, linewidth=linewidth)

        plt.xlim(-40, 40)
        plt.ylim(-85, 60)
        plt.title(f"Pose Graph Iteration {last_iteration}")
        plt.grid()

        image_file = os.path.join(output_dir, f"pose_graph_{last_iteration}.png")
        plt.savefig(image_file)
        image_files.append(image_file)
        plt.close()

    gif_file = os.path.join(output_dir, "pose_graphs.gif")
    with imageio.get_writer(gif_file, mode="I", duration=1) as writer:
        for image_file in image_files:
            writer.append_data(imageio.imread(image_file))


if __name__ == "__main__":
    draw_pose_graph("pose_graph_info.json", "pose_graph_res")
