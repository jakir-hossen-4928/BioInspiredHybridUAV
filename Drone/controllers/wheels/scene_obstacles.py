"""Read collision footprints from the kinematic Webots farm scene."""

from world_geometry import field, local_bounds, obstacle_from_solid


def flight_boxes(root_children, drone, padding=0.6):
    px, py, pz = drone
    boxes = []
    for i in range(root_children.getCount()):
        node = root_children.getMFNode(i)
        if node.getBaseTypeName() != 'Solid':
            continue
        x, y, _ = node.getPosition()
        if abs(x - px) > 20 or abs(y - py) > 20:
            continue
        bounding = field(node, 'boundingObject')
        geometry = bounding.getSFNode() if bounding else None
        if geometry is None:
            continue
        try:
            obstacle = obstacle_from_solid(node, local_bounds(geometry), 'Live Webots solid')
        except ValueError:
            continue  # shortcut: primitive collision bodies only until mesh bounds are available.
        if obstacle.pop('bottom') <= pz + padding and obstacle.pop('top') >= max(.2, pz - padding):
            boxes.append({key: round(value, 2) if key != 'name' else value
                          for key, value in obstacle.items()})
    return boxes
