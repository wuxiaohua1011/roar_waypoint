"""
The purpose of this file is to take in a txt file in containing data
x,y,z,roll,pitch,yaw or x,y,z
...
and visualize the track
"""
from dis import dis
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np
import plotly.graph_objects as go
from glob import glob
import math
from plotly.subplots import make_subplots
import os
import pandas as pd
import matplotlib.pyplot as plt


def read_txt(file_path: Path) -> List[List[float]]:
    if file_path.exists() is False:
        raise FileNotFoundError(f"{file_path} is not found. Please double check")
    file = file_path.open("r")
    result: List[List[float]] = []
    for line in file.readlines():
        try:
            x, y, z = line.split(sep=",")
        except Exception as e:
            data = line.split(sep=",")
            x, y, z = data[:3]

        result.append([float(x), float(y), float(z)])
    return result


def visualize_track_data(track_data: List[List[float]], file_name: Optional[Path]):
    print(f"Visualizing [{len(track_data)}] data points")

    track_data = np.asarray(track_data)
    # times = [i for i in range(len(track_data))]
    Xs = track_data[:, 0]
    Ys = track_data[:, 1]
    Zs = track_data[:, 2]

    fig = make_subplots(
        rows=3, cols=2, subplot_titles=["Xs", "X vs Y", "Ys", "Y vs Z", "Zs", "X vs Z"]
    )
    fig.update_layout(title=f"Data file path: {file_name.as_posix()}")
    fig.add_trace(
        go.Scatter(
            y=Xs,
            mode="markers",
            marker=dict(color="Red"),
            name="Xs",
        ),
        row=1,
        col=1,
    )

    fig.add_trace(
        go.Scatter(y=Ys, mode="markers", marker=dict(color="Blue"), name="Ys"),
        row=2,
        col=1,
    )

    fig.add_trace(
        go.Scatter(y=Zs, mode="markers", marker=dict(color="Green"), name="Zs"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=Xs, y=Ys, mode="markers", marker=dict(color="Black"), name="X vs Y"
        ),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(
            x=Xs, y=Zs, mode="markers", marker=dict(color="Orange"), name="X vs Z"
        ),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(
            x=Ys, y=Zs, mode="markers", marker=dict(color="Yellow"), name="Y vs Z"
        ),
        row=3,
        col=2,
    )

    fig.show()


def visualize_tracks(
    data_dir: Path = Path("../ROAR_Gym/data"),
    width: int = 1000,
    height: int = 1000,
    regex: str = "transforms_*",
):
    # load tracks
    tracks: Dict[str, List[List[float]]] = dict()  # track_name -> track waypoints
    paths = sorted(glob((data_dir / regex).as_posix()), key=os.path.getmtime)
    print(f"Visualizing [{len(paths)}] tracks")
    for file_path in paths:
        file_path = Path(file_path)
        track = read_txt(file_path=file_path)
        tracks[file_path] = track

    rows = cols = math.ceil(
        math.sqrt(len(tracks))
    )  # calculate how many rows and cols i need
    fig = make_subplots(rows=rows, cols=cols)
    track_names = list(tracks.keys())
    index = 0
    for row in range(1, rows + 1):
        for col in range(1, cols + 1):
            track_data = np.asarray(tracks[track_names[index]])
            displayed_name: str = track_names[index]
            displayed_name = Path(displayed_name).name
            fig.add_trace(
                go.Scatter(
                    x=track_data[:, 0],
                    y=track_data[:, 1],
                    mode="markers",
                    name=f"{displayed_name}",
                ),
                row=row,
                col=col,
            )
            index += 1
            if index == len(track_names):
                break
        if index == len(track_names):
            break

    fig.update_layout(
        height=height,
        width=width,
        title_text=f"All Plots in [{data_dir}] directory matching [{regex}]",
    )
    fig.show()


def visualize_tracks_together(
    data_dir: Path = Path("../ROAR_Gym/data"),
    width: int = 1000,
    height: int = 1000,
    regex: str = "transforms_*",
):
    # load tracks
    tracks: Dict[str, List[List[float]]] = dict()  # track_name -> track waypoints
    paths = sorted(glob((data_dir / regex).as_posix()), key=os.path.getmtime)
    print(f"Visualizing [{len(paths)}] tracks")
    for file_path in paths:
        file_path = Path(file_path)
        track = read_txt(file_path=file_path)
        tracks[file_path.name] = track

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    for name, track in tracks.items():
        track = np.array(track)
        ax1.scatter(track[:, 0], track[:, 1], s=10, marker="s", label=name)
    plt.legend(loc="upper left")
    plt.show()


def find_largest_dist_apart(path: Path):
    track_data = read_txt(file_path=Path(path))
    largest_dist_apart = 0
    for i in range(1, len(track_data)):
        prev = track_data[i - 1]
        curr = track_data[i]
        prevx, prevy, prevz = prev
        currx, curry, currz = curr
        dist = np.sqrt((prevx - currx) ** 2 + (prevy - curry) ** 2)
        if largest_dist_apart < dist:
            largest_dist_apart = dist
    return largest_dist_apart


def smooth_track(path: Path, threshold=1):
    track_data = read_txt(file_path=Path(path))
    largest_dist_apart = 0
    result = [track_data[0]]
    for i in range(1, len(track_data)):
        prev = track_data[i - 1]
        curr = track_data[i]
        prevx, prevy, prevz = prev
        currx, curry, currz = curr
        dist = np.sqrt((prevx - currx) ** 2 + (prevy - curry) ** 2)
        if dist > threshold:
            result.append(track_data[i])
    print(f"Writing {len(result)} points")
    directory = path.parent
    output_path = str(path.name).split(".")[0] + "_smoothed.txt"
    output_path = directory / output_path
    file = output_path.open("w")
    for r in result:
        file.write(",".join([str(item) for item in r]) + "\n")
    file.close()


if __name__ == "__main__":
    file_name = Path(
        "/home/roar/Desktop/projects/roar-indy-ws/data/waypoints/30_06_2022_13_10_59.txt"
    )
    track_data: List[List[float]] = read_txt(file_name)
    # track_data = swapCols(track_data)
    # save(track_data)

    visualize_track_data(track_data=track_data, file_name=file_name)
    # visualize_tracks(data_dir=Path("/home/roar/roar/roar_ros/data/waypoints"), regex="*")
    # visualize_tracks_together(data_dir=Path("/home/roar/roar/roar_ros/data/waypoints"), regex="*.txt")
    # smooth_track(file_name)
