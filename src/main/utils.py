import load000setup
import math
import parameters

# method that interpolates between two points
def interpolateBetweenTwoPoints(node1, node2, steps):
    x1 = node1['x']
    x2 = node2['x']
    y1 = node1['y']
    y2 = node2['y']
    deltaX = (x2 - x1)
    deltaY = (y2 - y1)

    outputNodeArray = []
    for i in range(0, steps):
        outputNodeArray.append({ 'x': (x1 + ((deltaX/steps)*i)), 'y': (y1 + ((deltaY/steps)*i))})

    return outputNodeArray

# method that interpolates between two points
def interpolate(x_start, y_start, x_end, y_end, steps):
    output = []
    deltaX = (x_end - x_start)
    deltaY = (y_end - y_start)

    for i in range(0, steps):
        output.append({ 'x': (x_start + ((deltaX/steps)*i)), 'y': (y_start + ((deltaY/steps)*i))})

    return output

# get direction of nearest lane (starting line to other end line direction)
def getDirectionOfNearestLane(annotation):
    nearestLane = load000setup.getClosestLaneOfSampleAnnotation(annotation)
    start_x = nearestLane[0]['start_pose'][0]
    start_y = nearestLane[0]['start_pose'][1]
    end_x = nearestLane[0]['end_pose'][0]
    end_y = nearestLane[0]['end_pose'][1]

    delta_x = (end_x - start_x)
    delta_y = (end_y - start_y)

    distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    direction = ((delta_y / delta_x)/distance)

    return direction

def getDeltaXAndDeltaYOfNearestLane(annotation):
    nearestLane = load000setup.getClosestLaneOfSampleAnnotation(annotation)
    start_x = nearestLane[0]['start_pose'][0]
    start_y = nearestLane[0]['start_pose'][1]
    end_x = nearestLane[0]['end_pose'][0]
    end_y = nearestLane[0]['end_pose'][1]

    delta_x = (end_x - start_x)
    delta_y = (end_y - start_y)

    return [delta_x, delta_y]

# get direction of two points
def getOrthogonalDirectionOfNearestLane(annotation):
    direction = getDirectionOfNearestLane(annotation)
    
    orthogonalDirection = (-1)*(direction)

    return orthogonalDirection


# get direction of two points
def getDirectionOfTwoNodes(node1, node2):
    distance = calculateDistanceBetweenTwoNodes(node1, node2)
    start_x = node1['x']
    start_y = node1['y']
    end_x = node2['x']
    end_y = node2['y']

    delta_x = (end_x - start_x)
    delta_y = (end_y - start_y)
    direction = ((delta_y / delta_x)/distance)

    return direction

def calculateDistanceBetweenTwoNodes(node1, node2):
    start_x = node1['x']
    start_y = node1['y']
    end_x = node2['x']
    end_y = node2['y']

    delta_x = end_x - start_x
    delta_y = end_y - start_y

    distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    return distance

# check if directions are roughly the same:
def compareDirections(direction1, direction2):
    delta_direction = abs(direction1 - direction2)
    delta_opposite_direction = abs(direction1 + direction2)

    if delta_direction < parameters.same_direction_threshold:
        return 'same_direction'
    if delta_opposite_direction < parameters.opposite_direction_threshold:
        return 'opposite_direction'
    else:
        return ''