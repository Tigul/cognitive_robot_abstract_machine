from __future__ import annotations

import pathlib
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    Optional,
    Sequence,
    Tuple,
    TYPE_CHECKING,
)
from dataclasses import dataclass, field

import networkx as nx
from bokeh.io import output_file
from rustworkx.rustworkx import PyDiGraph

if TYPE_CHECKING:
    from bokeh.document import Document
    from bokeh.models import ColumnDataSource, Div, GraphRenderer
    from bokeh.server.server import Server


@dataclass
class GraphVisualizer:
    """
    Handles the interactive visualization of a rustworkx graph using Bokeh.
    Supports dynamic updates by periodically checking for graph changes.
    """

    graph: Any
    node_params: Optional[Dict[int, Dict[str, Any]]] = None
    node_label: Optional[Callable[[int, Any], str]] = None
    attributes: Optional[Sequence[str]] = None
    layout: str = "bfs"
    start: Optional[int] = None
    title: str = "Rustworkx Graph"
    width: int = 1200
    height: int = 800
    update_interval: int = 1000

    def _build_bokeh_app(self, doc: Document) -> None:
        # Local imports to keep dependency optional
        from bokeh.layouts import row
        from bokeh.models import (
            HoverTool,
            NodesAndLinkedEdges,
            TapTool,
            CustomJS,
            Div,
        )
        from bokeh.plotting import figure, from_networkx

        nx_graph = build_nx_graph(
            self.graph, self.node_params, self.attributes, self.node_label
        )
        positions = calculate_layout_positions(self.layout, nx_graph, self.start)

        plot = figure(
            title=self.title,
            x_axis_location=None,
            y_axis_location=None,
            width=self.width,
            height=self.height,
            toolbar_location="below",
            background_fill_color="#efefef",
        )
        plot.grid.grid_line_color = None

        renderer = from_networkx(nx_graph, positions)
        renderer.node_renderer.glyph.update(size=18, fill_color="#79a6d2")

        hover = HoverTool(tooltips=[("label", "@label")])
        plot.add_tools(hover, TapTool())
        renderer.selection_policy = NodesAndLinkedEdges()
        renderer.inspection_policy = NodesAndLinkedEdges()

        info_panel = Div(
            text="<b>Click a node to see its parameters</b>",
            width=400,
            height=self.height,
        )

        node_source = renderer.node_renderer.data_source
        self._update_node_source(node_source, nx_graph)

        callback = CustomJS(
            args=dict(source=node_source, panel=info_panel),
            code="""
                const indices = source.selected.indices;
                if (indices.length === 0) {
                    panel.text = "<b>Click a node to see its parameters</b>";
                    return;
                }
                const index = indices[0];
                const label = source.data['label'][index];
                const parameters = source.data['param_text'][index] || '';
                panel.text = `<div><h3 style=\"margin:0 0 8px 0;\">${label}</h3>${parameters}</div>`;
            """,
        )
        node_source.selected.js_on_change("indices", callback)

        plot.renderers.append(renderer)

        last_state = {
            "node_indices": set(self.graph.node_indices()),
            "edges": set(self.graph.edge_list()),
            "node_data": {i: self.graph[i] for i in self.graph.node_indices()},
            "edge_data": {
                edge: self.graph.get_edge_data(*edge) for edge in self.graph.edge_list()
            },
        }

        def update_callback() -> None:
            nonlocal last_state
            try:
                current_node_indices = set(self.graph.node_indices())
                current_edges = set(self.graph.edge_list())
                current_node_data = {
                    i: self.graph[i] for i in self.graph.node_indices()
                }
                current_edge_data = {
                    edge: self.graph.get_edge_data(*edge)
                    for edge in self.graph.edge_list()
                }

                if (
                    current_node_indices != last_state["node_indices"]
                    or current_edges != last_state["edges"]
                    or current_node_data != last_state["node_data"]
                    or current_edge_data != last_state["edge_data"]
                ):
                    last_state["node_indices"] = current_node_indices
                    last_state["edges"] = current_edges
                    last_state["node_data"] = current_node_data
                    last_state["edge_data"] = current_edge_data
                    self._update_plot(renderer, node_source)
            except Exception:
                # Avoid crashing the periodic callback on transient errors
                pass

        doc.add_periodic_callback(update_callback, self.update_interval)
        doc.add_root(row(plot, info_panel))

    def _update_node_source(self, source: ColumnDataSource, nx_graph: nx.Graph) -> None:
        indices = list(source.data.get("index", []))
        labels = [nx_graph.nodes[i].get("label", str(i)) for i in indices]
        parameters = [nx_graph.nodes[i].get("param_text", "") for i in indices]
        source.data.update({"label": labels, "param_text": parameters})

    def _update_plot(
        self, renderer: GraphRenderer, node_source: ColumnDataSource
    ) -> None:
        from bokeh.plotting import from_networkx

        new_nx_graph = build_nx_graph(
            self.graph, self.node_params, self.attributes, self.node_label
        )
        new_positions = calculate_layout_positions(
            self.layout, new_nx_graph, self.start
        )
        new_renderer = from_networkx(new_nx_graph, new_positions)

        # Update existing data sources to refresh the plot
        new_node_data = dict(new_renderer.node_renderer.data_source.data)
        indices = list(new_node_data.get("index", []))
        labels = [new_nx_graph.nodes[i].get("label", str(i)) for i in indices]
        parameters = [new_nx_graph.nodes[i].get("param_text", "") for i in indices]

        node_source.data = {
            **new_node_data,
            "label": labels,
            "param_text": parameters,
        }
        renderer.edge_renderer.data_source.data = dict(
            new_renderer.edge_renderer.data_source.data
        )

        # Update layout positions
        renderer.layout_provider.graph_layout = new_positions

    def show(self) -> None:
        """Launch the Bokeh server and show the plot."""
        from bokeh.server.server import Server
        import threading

        server = Server({"/": self._build_bokeh_app}, port=0)
        server.start()
        server.show("/")

        # Run the server loop in a separate thread so it doesn't block the caller
        thread = threading.Thread(target=server.io_loop.start, daemon=True)
        thread.start()


def create_ordered_graph(plan):
    ordered_graph = PyDiGraph(multigraph=False)
    mapping = {}

    for node in plan.nodes:
        mapping[node.index] = ordered_graph.add_node(node)
    for node in plan.nodes:
        for child in node.children:
            ordered_graph.add_edge(mapping[node.index], mapping[child.index], None)
    return ordered_graph


def plot_rustworkx_interactive(
    graph: Any,
    *,
    node_params: Optional[Dict[int, Dict[str, Any]]] = None,
    node_label: Optional[Callable[[int, Any], str]] = None,
    attributes: Optional[Sequence[str]] = None,
    layout: str = "bfs",
    start: Optional[int] = None,
    title: str = "Rustworkx Graph",
    width: int = 1200,
    height: int = 800,
):
    """
    Plot an interactive visualization of a rustworkx graph.

    - Click on a node to show its parameters in a side panel.
    - Hover shows the node label.
    - The plot is dynamically updated when the given graph is changing.

    Parameters
    ----------
    graph:
        A rustworkx.PyGraph or rustworkx.PyDiGraph instance.
    node_params:
        Optional mapping from node index to a dict of parameters to display when
        the node is clicked. If not provided and the node payload is a dict,
        those items will be used. If provided together with ``attributes``, the
        displayed parameters will be filtered to the given attribute names.
    node_label:
        Optional callable that takes (index, payload) and returns a label string
        for the node. By default it tries to use ``payload.get('label')`` or
        ``str(payload)``.
    attributes:
        Optional list of attribute names to show from the parameters. Ignored if
        parameters are not dict-like.
    layout:
        Layout algorithm to use: "spring", "kamada_kawai", or "bfs".
    start:
        Optional start node index for "bfs" layout.
    title:
        Plot title.
    width, height:
        Figure size in pixels.

    Notes
    -----
    This function imports bokeh lazily so that it does not add a hard runtime
    dependency unless you call it. Install with `pip install bokeh`.
    """

    # Local imports to keep dependency optional at import time.
    try:
        import networkx as nx
        from bokeh.layouts import row
        from bokeh.models import (
            ColumnDataSource,
            Div,
            HoverTool,
            NodesAndLinkedEdges,
            TapTool,
            CustomJS,
        )
        from bokeh.plotting import figure, from_networkx
        from bokeh.server.server import Server
    except Exception as exc:  # pragma: no cover - informative error only if used
        raise RuntimeError(
            "plot_rustworkx_interactive requires bokeh and networkx. Install with 'pip install bokeh networkx'."
        ) from exc

    visualizer = GraphVisualizer(
        graph=graph,
        node_params=node_params,
        node_label=node_label,
        attributes=attributes,
        layout=layout,
        start=start,
        title=title,
        width=width,
        height=height,
    )
    visualizer.show()


def calculate_layout_positions(
    layout: str, nx_g: nx.Graph, start: Optional[int] = None
) -> Dict[int, Tuple[float, float]]:
    """
    Calculates node positions based on the selected layout.
    :param layout: Layout name, e.g. "spring", "kamada_kawai", "bfs
    :param nx_g: networkx graph
    :param start: Optional start node index for "bfs" layout.
    :return: A dictionary mapping node indices to 2d coordinates.
    """
    # Choose layout
    if layout == "spring":
        pos = nx.spring_layout(nx_g, seed=42)
    elif layout == "kamada_kawai":
        pos = nx.kamada_kawai_layout(nx_g)
    elif layout == "bfs":
        if start is None and len(nx_g.nodes) > 0:
            start = list(nx_g.nodes)[0]
        try:
            pos = nx.bfs_layout(nx_g, start=start)
        except nx.NetworkXError:
            pos = nx.spring_layout(nx_g, seed=42)
    else:
        pos = nx.spring_layout(nx_g, seed=42)
    return pos


def build_nx_graph(graph: PyDiGraph, node_params, attributes, node_label) -> nx.Graph:
    """Convert a rustworkx graph to a networkx graph."""
    # Build a NetworkX graph from rustworkx graph
    is_directed = getattr(graph, "is_directed", lambda: True)()
    nx_g = nx.DiGraph() if is_directed else nx.Graph()

    # rustworkx nodes are indexed 0..n-1. Access via graph.nodes(), graph.node_indices() or graph.num_nodes()
    # We'll iterate over range(num_nodes) and get payload via graph[node]
    num_nodes = graph.num_nodes()

    # Prepare label/params for each node
    attributes = list(attributes) if attributes is not None else None

    for i in range(num_nodes):
        payload = graph[i]
        # Label
        if node_label is not None:
            label = node_label(i, payload)
        else:
            label = None
            if isinstance(payload, dict) and "label" in payload:
                label = str(payload.get("label"))
            if label is None:
                label = str(payload)
        # Parameters
        params = None
        if node_params is not None:
            params = node_params.get(i)
        else:
            params = _object_params_with_properties(payload)
        # Filter attributes if requested
        if attributes is not None and isinstance(params, dict):
            params = {k: params.get(k) for k in attributes if k in params}
        # Attach as node attributes
        nx_g.add_node(
            i,
            label=label,
            param_text=_format_params(params),
        )

    # Add edges
    for u, v in graph.edge_list():
        nx_g.add_edge(u, v)

    return nx_g


def _object_params_with_properties(payload: Any) -> Optional[Dict[str, Any]]:
    """
    Build a parameter dictionary from a node payload by combining:
    - public attributes from payload.__dict__ (if present)
    - readable @property attributes defined on the payload's class
    - if payload is a dict, return it (excluding 'label')

    Private attributes (starting with '_') and the key 'label' are excluded.
    Values that raise on access are skipped. Callables are skipped.
    """
    # If the payload is already a dict, filter and return it.
    if isinstance(payload, dict):
        return {k: v for k, v in payload.items() if k != "label"}

    if payload is None:
        return None

    params: Dict[str, Any] = {}

    # Collect from __dict__ if available
    try:
        if hasattr(payload, "__dict__") and isinstance(
            getattr(payload, "__dict__", None), dict
        ):
            for k, v in vars(payload).items():
                if k.startswith("_") or k == "label":
                    continue
                # Avoid adding callables
                try:
                    is_callable = callable(v)
                except Exception:
                    is_callable = False
                if not is_callable:
                    params[k] = v
    except Exception:
        pass

    params.update(_collect_properties(payload))

    return params if params else None


def _collect_properties(payload) -> Dict[str, Any]:
    params = {}
    # Collect readable @property attributes on the class
    try:
        import inspect

        cls = type(payload)
        for name, member in inspect.getmembers(cls):
            if not isinstance(member, property):
                continue
            if name.startswith("_") or name == "label":
                continue
            if name in params:
                continue  # do not overwrite explicit attributes
            # Access property value safely
            try:
                value = getattr(payload, name)
            except Exception:
                continue
            # Skip callables
            try:
                if callable(value):
                    continue
            except Exception:
                pass
            params[name] = value
    except Exception:
        # If inspection fails, just ignore properties
        pass
    return params


def _format_params(params: Optional[Dict[str, Any]]) -> str:
    """Return HTML for parameter dict suitable for the side panel."""
    if not params:
        return "<i>No parameters</i>"
    try:
        items = []
        for k, v in params.items():
            items.append(
                f"<tr><td style='padding-right:8px; white-space:nowrap;'><b>{k}</b></td><td>{_escape_html(v)}</td></tr>"
            )
        return "<table>" + "".join(items) + "</table>"
    except Exception:
        return f"<pre>{_escape_html(params)}</pre>"


def _escape_html(value: Any) -> str:
    try:
        s = str(value)
    except Exception:
        s = repr(value)
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#39;")
    )
