import queue
import quadtreemap

def _get_movements_4n(qtm, tile):

    neighborList = []
    neighborList.append(qtm.quadtree.tileIntersect(quadtreemap.BoundingBox(tile.boundary.x0-1, tile.boundary.y0,
                                                                    tile.boundary.width+2, tile.boundary.height)))
    neighborList.append(qtm.quadtree.tileIntersect(quadtreemap.BoundingBox(tile.boundary.x0, tile.boundary.y0-1,
                                                                    tile.boundary.width, tile.boundary.height+2)))
    movements = [(til, quadtreemap.Point.disOf2Points(tile.getCenter(), til.getCenter())) for til in neighborList]
    return movements

def _get_movements_8n(qtm: quadtreemap.QuadTreeMap , tile: quadtreemap.Tile):

    neighborList = qtm.quadtree.tileIntersect(quadtreemap.BoundingBox(tile.boundary.x0-1, tile.boundary.y0-1,
                                            tile.boundary.width+2, tile.boundary.height+2))
    movements = [(til, quadtreemap.Point.disOf2Points(tile.getCenter(), til.getCenter())) for til in neighborList]
    return movements


def heuristic(node, goal):

    return quadtreemap.Point.disOf2Points(node.getCenter(), goal.getCenter())

def a_star_quadtree(start_m, goal_m, qtm, movement='8n', occupancy_cost_factor=3):
    path_record = {}
    candidates = queue.PriorityQueue()


    start = qtm.quadtree.searchTileByIdx(quadtreemap.Point(start_m[0], start_m[1]))
    goal = qtm.quadtree.searchTileByIdx(quadtreemap.Point(goal_m[0], goal_m[1]))


    if not start or start.tile_points:
        raise Exception('Start node is not traversable')
    if not goal or goal.tile_points:
        raise Exception('Goal node is not traversable')

    candidates.put((0, None, start))  # store (distance, previous-tile, current-tile)
    while candidates:
        dis, prev_node, curr_node = candidates.get()
        if curr_node == goal:
            path_record[curr_node] = prev_node
            break
        if curr_node in path_record:
            continue
        path_record[curr_node] = prev_node


        if movement == '4N':
            movements = _get_movements_4n(qtm, curr_node)
        elif movement == '8N':
            movements = _get_movements_8n(qtm, curr_node)
        else:
            raise ValueError('Unknown movement')


        for til, deltacost in movements:


            if til.tile_points:
                continue
            if til not in path_record:

                cost = dis + deltacost + heuristic(til, goal)
                candidates.put((cost, curr_node, til))


    path = []
    path_idx = []

    if goal in path_record:
        node = goal
        while node:
            path_idx.append(node)
            node = path_record[node]
        path.reverse()
        path_idx.reverse()

    return path, path_idx
