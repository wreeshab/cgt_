import random
import matplotlib.pyplot as plt
import networkx as nx

def check_graphic_sequence(seq):
    seq = sorted(seq, reverse=True)
    while seq and seq[0] == 0:
        seq = seq[1:]
    if not seq:
        return True
    if seq[0] < 0 or seq[0] >= len(seq):
        return False
    d = seq[0]
    seq = seq[1:]
    for i in range(d):
        seq[i] -= 1
    return check_graphic_sequence(seq)

def apply_havel_hakimi(seq):
    seq = sorted(seq, reverse=True)
    graph = nx.Graph()
    
    if not check_graphic_sequence(seq):
        raise ValueError("The sequence is not graphic")

    node_list = list(range(len(seq)))  # Assign node IDs
    while seq:
        seq = sorted(seq, reverse=True)
        d = seq[0]
        node = node_list[0]
        seq = seq[1:]
        node_list = node_list[1:]
        if d > len(seq):
            raise ValueError("Invalid graphic sequence")
        for i in range(d):
            graph.add_edge(node, node_list[i], weight=random.randint(1, 10))
            seq[i] -= 1
        seq = [x for x in seq if x > 0]
        node_list = [node_list[i] for i in range(len(seq))]  # Keep only nodes with remaining degrees
    
    return graph

def visualize_graph(graph, title, edges=None):
    pos = nx.spring_layout(graph)
    plt.figure(figsize=(8, 6))
    nx.draw(graph, pos, with_labels=True, node_color="lightblue", edge_color="gray", node_size=500, font_size=12)
    edge_labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels)
    if edges:
        nx.draw_networkx_edges(graph, pos, edgelist=edges, edge_color="red", width=2)
    plt.suptitle(title, fontsize=16)
    plt.show()

def check_eulerian_type(graph):
    odd_degree_nodes = [v for v, d in graph.degree() if d % 2 == 1]
    if len(odd_degree_nodes) == 0:
        return "Circuit"
    elif len(odd_degree_nodes) == 2:
        return "Path"
    else:
        return None

def detect_bridge(g, u, v):
    g_copy = g.copy()
    g_copy.remove_edge(u, v)
    return nx.number_connected_components(g_copy) > 1

def fleury_traversal(graph, start_node):
    g = graph.copy()
    euler_path = []
    
    def valid_edge(u):
        for v in g.neighbors(u):
            if nx.number_connected_components(g) == 1 or not detect_bridge(g, u, v):
                return v
        return None

    current_node = start_node
    while g.number_of_edges() > 0:
        next_node = valid_edge(current_node)
        if next_node is None:
            break
        euler_path.append((current_node, next_node))
        g.remove_edge(current_node, next_node)
        current_node = next_node

    return euler_path

def prim_algorithm(graph):
    mst_edges = []
    visited = {list(graph.nodes)[0]}
    edges = [(weight, u, v) for u, v, weight in graph.edges(data='weight')]
    edges.sort()

    while len(visited) < len(graph.nodes):
        for weight, u, v in edges:
            if u in visited and v not in visited:
                visited.add(v)
                mst_edges.append((u, v))
                break
            elif v in visited and u not in visited:
                visited.add(u)
                mst_edges.append((v, u))
                break
    return mst_edges

def calc_fundamental_cutsets(graph, mst_edges):
    cutsets = []
    non_mst_edges = [edge for edge in graph.edges if edge not in mst_edges]
    for edge in non_mst_edges:
        g_copy = nx.Graph(mst_edges)
        g_copy.add_edge(*edge)
        cutset = list(nx.minimum_edge_cut(g_copy))
        cutsets.append(cutset)
    return cutsets

def calc_fundamental_circuits(graph, mst_edges):
    circuits = []
    non_mst_edges = [edge for edge in graph.edges if edge not in mst_edges]
    for edge in non_mst_edges:
        mst_copy = nx.Graph(mst_edges)
        mst_copy.add_edge(*edge)
        try:
            circuit = nx.find_cycle(mst_copy, source=edge[0])
            circuits.append(circuit)
        except nx.exception.NetworkXNoCycle:
            continue
    return circuits

def compute_connectivity(graph):
    edge_connectivity = nx.edge_connectivity(graph)
    vertex_connectivity = nx.node_connectivity(graph)
    k_connectivity = min(edge_connectivity, vertex_connectivity)
    
    return edge_connectivity, vertex_connectivity, k_connectivity

def run_dijkstra(graph, start_node):
    paths = nx.single_source_dijkstra_path(graph, start_node, weight='weight')
    return paths

sequence=eval(input("Enter the sequence as List"))
n=len(sequence)

graph = apply_havel_hakimi(sequence)

euler_type = check_eulerian_type(graph)

if euler_type:
    if euler_type == "Circuit":
        start_node = list(graph.nodes)[0]
    elif euler_type == "Path":
        start_node = [v for v, d in graph.degree() if d % 2 == 1][0]
    
    euler_path = fleury_traversal(graph, start_node)
else:
    euler_path = None

mst_edges = prim_algorithm(graph)

fundamental_cutsets = calc_fundamental_cutsets(graph, mst_edges)
print(f"Fundamental Cutsets: {fundamental_cutsets}")

fundamental_circuits = calc_fundamental_circuits(graph, mst_edges)
print(f"Fundamental Circuits: {fundamental_circuits}")

edge_conn, vertex_conn, k_conn = compute_connectivity(graph)

start_node_dijkstra = random.choice(list(graph.nodes))
shortest_paths = run_dijkstra(graph, start_node_dijkstra)

visualize_graph(graph, "Original Graph")
visualize_graph(graph, "Eulerian Path", edges=euler_path)
visualize_graph(graph, "Minimum Spanning Tree (MST)", edges=mst_edges)

flat_cutsets = [edge for cut in fundamental_cutsets for edge in cut if len(edge) == 2]
visualize_graph(graph, "Fundamental Cutsets", edges=flat_cutsets)

flat_circuits = [edge for circ in fundamental_circuits for edge in circ if len(edge) == 2]
visualize_graph(graph, "Fundamental Circuits", edges=flat_circuits)

highlighted_edges = [(u, v) for path in shortest_paths.values() for u, v in zip(path[:-1], path[1:])]
visualize_graph(graph, f"Dijkstra's Shortest Paths from Node {start_node_dijkstra}", edges=highlighted_edges)

print(f"Edge Connectivity: {edge_conn}")
print(f"Vertex Connectivity: {vertex_conn}")
print(f"K-Connectivity: {k_conn}")
