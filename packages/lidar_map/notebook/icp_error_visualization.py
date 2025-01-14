import json

import pandas as pd
import plotly.express as px


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


def plot_icp_errors_interactive(icp_errors, iteration):
    icp_errors.sort(key=lambda x: x[0])
    errors = [error[0] for error in icp_errors]
    lengths = [abs(error[1] - error[2]) for error in icp_errors]

    df = pd.DataFrame({"Length": lengths, "Error": errors})

    fig = px.bar(
        df,
        x="Length",
        y="Error",
        title=f"ICP Errors iteration={iteration}",
        labels={"Length": "Length of Edge", "Error": "ICP Error"},
        text="Error",
        color="Error",
        color_continuous_scale=px.colors.sequential.Blues,
    )

    fig.update_traces(texttemplate="%{text:.2f}", textposition="outside")

    fig.update_yaxes(type="log")
    fig.update_layout(height=600)

    fig.show()


def plot_icp_errors_histogram(icp_errors, iteration):
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

    fig.update_yaxes(type="log")
    fig.update_layout(height=600)

    fig.show()


if __name__ == "__main__":
    json_path = "pose_graph_info.json"
    iteration = 9
    icp_errors = parse_json_and_extract_icp_errors(json_path, iteration)

    plot_icp_errors_interactive(icp_errors, iteration)
    plot_icp_errors_histogram(icp_errors, iteration)
