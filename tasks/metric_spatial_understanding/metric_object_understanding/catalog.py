# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""MOU is one task within Metric Spatial Understanding, with eleven cases."""

CATEGORY = "metric_spatial_understanding"
TASK = "metric_object_understanding"
REGISTRY_ID = f"{CATEGORY}/{TASK}"
CONDITIONS = ("same_shape", "cross_shape", "shape_conditioned")
# Keep geometry definitions readable for historical episodes, but do not
# generate demonstrations with a grasp that failed physical qualification.
AUTONOMOUS_EXCLUSIONS = {
    "cone": "The overhead Panda grasp slips on tapered surfaces under the allowed mass randomization."
}
CASES = {
    "length": dict(
        id="MOU-1.1", family="physical_dimensions", shapes=["rectangular_bar", "box"], query={"length": (14, "cm")}
    ),
    "width": dict(
        id="MOU-1.2",
        family="physical_dimensions",
        shapes=["rectangular_plate", "box", "rectangular_container"],
        query={"width": (6, "cm")},
    ),
    "height": dict(
        id="MOU-1.3", family="physical_dimensions", shapes=["cylinder", "box", "cone"], query={"height": (12, "cm")}
    ),
    "radius": dict(
        id="MOU-1.4",
        family="physical_dimensions",
        shapes=["cylinder", "sphere", "circular_disk", "puck"],
        query={"radius": (2.9, "cm")},
    ),
    "diameter": dict(
        id="MOU-1.5",
        family="physical_dimensions",
        shapes=["sphere", "cylinder", "circular_disk"],
        query={"diameter": (5.8, "cm")},
    ),
    "area": dict(
        id="MOU-2.1",
        family="geometric_properties",
        shapes=["circular_disk", "rectangular_plate", "box"],
        query={"footprint_area": (35, "cm2")},
    ),
    "volume": dict(
        id="MOU-2.2", family="geometric_properties", shapes=["box", "sphere", "cone"], query={"volume": (160, "cm3")}
    ),
    "capacity": dict(
        id="MOU-2.3",
        family="geometric_properties",
        shapes=["cylindrical_container", "rectangular_container"],
        query={"capacity": (250, "cm3")},
    ),
    "multi_dimension": dict(
        id="MOU-3.1",
        family="structured_objects",
        shapes=["box", "handled_tool"],
        query={"length": (14, "cm"), "width": (5, "cm"), "height": (4, "cm")},
    ),
    "component_dimension": dict(
        id="MOU-3.2",
        family="structured_objects",
        shapes=["handled_tool", "handled_paddle"],
        query={"handle_length": (9, "cm")},
    ),
    "nested_dimension": dict(
        id="MOU-3.3",
        family="structured_objects",
        shapes=["rectangular_container", "rectangular_tray"],
        query={"inner_width": (7, "cm"), "outer_width": (10, "cm")},
    ),
}

# Explicit authored query choices; values are never scaled or rounded after geometry generation.
QUERY_VALUES = {
    "length": [{"length": (v, "cm")} for v in (12, 14, 16)],
    "width": [{"width": (v, "cm")} for v in (5, 6, 7)],
    "height": [{"height": (v, "cm")} for v in (10, 12, 14)],
    "radius": [{"radius": (v, "mm")} for v in (25, 30, 35)],
    "diameter": [{"diameter": (v, "mm")} for v in (50, 60, 70)],
    "area": [{"footprint_area": (v, "cm2")} for v in (25, 30, 35)],
    "volume": [{"volume": (v, "cm3")} for v in (150, 175, 200)],
    "capacity": [{"capacity": (v, "cm3")} for v in (250, 300, 350)],
    "multi_dimension": [{"length": (v, "cm"), "width": (5, "cm"), "height": (4, "cm")} for v in (12, 14, 16)],
    "component_dimension": [{"handle_length": (v, "cm")} for v in (8, 9, 10)],
    "nested_dimension": [{"inner_width": (v, "cm"), "outer_width": (v + 3, "cm")} for v in (6, 7, 8)],
}
