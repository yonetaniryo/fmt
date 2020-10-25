from PIL import Image
import numpy as np
import networkx as nx
from fmt import FMTPlanner
import matplotlib.pyplot as plt


def load_map_design(img_path: str, size: list) -> np.ndarray:
    map_design = (np.asarray(Image.open(img_path).convert("L").resize(size)) /
                  255.).astype('int')

    return map_design


def visualize_result(map_design: np.ndarray, planner: FMTPlanner,
                     path_info: dict) -> None:
    plt.figure()
    plt.imshow(map_design, cmap="gray")
    nx.draw(planner.graph, [x[::-1] for x in planner.node_list],
            node_size=1,
            alpha=.5)
    path = path_info["path"]
    plt.plot(path[:, 1], path[:, 0], 'r-', lw=2)
