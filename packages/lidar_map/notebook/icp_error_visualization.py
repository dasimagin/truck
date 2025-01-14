import json

import pandas as pd
import plotly.express as px


def parse_json_and_extract_icp_errors(json_path):
    with open(json_path, "r") as file:
        data = json.load(file)

    icp_errors = []

    for iteration, iteration_data in data.items():
        edges = iteration_data.get("edges", [])

        for edge in edges:
            if edge.get("type") == "icp":
                icp_errors.append(
                    (edge["error_val"], edge["from_edge"], edge["to_edge"])
                    )

    return icp_errors


def plot_icp_errors_interactive(icp_errors):
    icp_errors.sort(key=lambda x: x[0])
    errors = [error[0] for error in icp_errors]
    lengths = [abs(error[1] - error[2]) for error in icp_errors]

    df = pd.DataFrame({"Length": lengths, "Error": errors})

    fig = px.bar(
        df,
        x="Length",
        y="Error",
        title="ICP Errors",
        labels={"Length": "Length of Edge", "Error": "ICP Error"},
        text="Error", color="Error", color_continuous_scale=px.colors.sequential.Blues
    )

    fig.update_traces(texttemplate="%{text:.2f}", textposition="outside")

    fig.update_yaxes(type="log")
    fig.update_layout(height=600)

    fig.show()


if __name__ == "__main__":
    json_path = "pose_graph_info.json"
    icp_errors = parse_json_and_extract_icp_errors(json_path)

    plot_icp_errors_interactive(icp_errors)
