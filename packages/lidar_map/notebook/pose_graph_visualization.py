import json
import os
import shutil

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.express as px
from PIL import Image


def draw_pose_graph(json_path, output_dir):
    with open(json_path, "r") as file:
        data = json.load(file)

    for key in data:
        last_iteration = key
        pose_graph_info = data[last_iteration]
        vertices = pose_graph_info["vertices"]
        edges = pose_graph_info["edges"]
        plt.figure(figsize=(6, 6))

        for vertex in vertices:
            x = vertex["x"]
            y = vertex["y"]
            theta = vertex["theta"]
            plt.plot(x, y, "ko")
            plt.text(x, y, str(vertex["id"]), fontsize=9, ha="right", va="bottom")
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
        image_file = os.path.join(
            output_dir, f"pose_graph_iteration_{last_iteration}.png"
        )
        plt.savefig(image_file)
        plt.close()


def parse_json_and_extract_icp_errors(json_path, iteration):
    with open(json_path, "r") as file:
        data = json.load(file)

    icp_errors = []
    iteration_data = data[str(iteration)]
    edges = iteration_data.get("edges", [])
    for edge in edges:
        if edge.get("type") == "icp":
            icp_errors.append((edge["error_val"], edge["from_edge"], edge["to_edge"]))

    return icp_errors


def plot_icp_errors_histogram(icp_errors, iteration, output_dir):
    sorted_icp_errors = sorted(
        icp_errors, key=lambda x: (min(x[1], x[2]), max(x[1], x[2]))
    )
    errors = [error[0] for error in sorted_icp_errors]
    edge_indices = list(range(1, len(errors) + 1))
    df = pd.DataFrame({"Edge Index": edge_indices, "Error": errors})

    fig = px.bar(
        df,
        x="Edge Index",
        y="Error",
        title=f"ICP Errors Histogram iteration={iteration}",
        labels={"Edge Index": "Edge Number", "Error": "ICP Error"},
        text="Error",
        color="Error",
        color_continuous_scale=px.colors.sequential.Reds,
    )

    fig.update_traces(texttemplate="%{text:.2f}", textposition="outside")
    fig.update_layout(height=600, width=600)
    fig.update_yaxes(range=[0, 0.02])
    fig.update_layout(height=600)
    output_file = os.path.join(
        output_dir, f"icp_errors_histogram_iteration_{iteration}.png"
    )
    fig.write_image(output_file)


def create_combined_images(icp_output_dir, pose_graph_output_dir, res_output_dir):
    combined_output_dir = os.path.join(res_output_dir, "combine")
    os.makedirs(combined_output_dir, exist_ok=True)

    iterations = [f"iteration_{i}" for i in range(len(os.listdir(icp_output_dir)))]
    for iteration in iterations:
        icp_image_path = os.path.join(
            icp_output_dir, f"icp_errors_histogram_{iteration}.png"
        )
        pose_graph_image_path = os.path.join(
            pose_graph_output_dir, f"pose_graph_{iteration}.png"
        )
        if os.path.exists(icp_image_path) and os.path.exists(pose_graph_image_path):
            icp_image = Image.open(icp_image_path)
            pose_graph_image = Image.open(pose_graph_image_path)

            combined_image = Image.new(
                "RGB",
                (
                    icp_image.width + pose_graph_image.width,
                    max(icp_image.height, pose_graph_image.height),
                ),
            )
            combined_image.paste(icp_image, (0, 0))
            combined_image.paste(pose_graph_image, (icp_image.width, 0))
            combined_image_path = os.path.join(
                combined_output_dir, f"combined_{iteration}.png"
            )
            combined_image.save(combined_image_path)


def create_gif_from_combined_images(combined_output_dir, gif_output_path):
    images = []
    for filename in sorted(os.listdir(combined_output_dir)):
        if filename.endswith(".png"):
            image_path = os.path.join(combined_output_dir, filename)
            images.append(imageio.imread(image_path))

    imageio.mimsave(gif_output_path, images, duration=6)


if __name__ == "__main__":
    res_output_dir = "res"
    json_path = "pose_graph_info.json"

    icp_output_dir = f"{res_output_dir}/icp_errors"
    pose_graph_output_dir = f"{res_output_dir}/pose_graph_res"
    os.makedirs(res_output_dir, exist_ok=True)
    os.makedirs(icp_output_dir, exist_ok=True)
    os.makedirs(pose_graph_output_dir, exist_ok=True)

    with open(json_path, "r") as file:
        data = json.load(file)

    for iteration in data.keys():
        icp_errors = parse_json_and_extract_icp_errors(json_path, iteration)
        if icp_errors:
            plot_icp_errors_histogram(icp_errors, iteration, icp_output_dir)

    draw_pose_graph(json_path, pose_graph_output_dir)
    create_combined_images(icp_output_dir, pose_graph_output_dir, res_output_dir)
    combined_output_dir = os.path.join(res_output_dir, "combine")
    gif_output_path = os.path.join(res_output_dir, "combined_animation.gif")
    create_gif_from_combined_images(combined_output_dir, gif_output_path)
    shutil.rmtree(icp_output_dir)
    shutil.rmtree(pose_graph_output_dir)
