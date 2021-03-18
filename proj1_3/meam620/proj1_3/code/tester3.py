
import numpy as np

waypoints = np.array([[0,0,0],
                      [1,1,1],
                      [2,2,2],
                      [4,4,4],
                      [3,2,3],
                      [4,4,4],
                      [5,5,5],
                      [6,6,6]])

def simplify_path(waypoints):

    length_waypoints = np.shape(waypoints)[0]
    good_waypoints = np.ones(length_waypoints)

    start = 0
    mid = 1
    end = 2
    while (end < length_waypoints):
        unit_vec1 = (waypoints[mid,:]-waypoints[start,:])/np.linalg.norm(waypoints[mid,:]-waypoints[start,:])
        unit_vec2 = (waypoints[end, :] - waypoints[mid, :]) / np.linalg.norm(waypoints[end, :] - waypoints[mid, :])
        if (np.linalg.norm(unit_vec2 - unit_vec1) < 0.001):
            good_waypoints[mid] = 0
            mid += 1
            end += 1
        else:
            start += 1
            while (good_waypoints[start] == 0):
                start += 1
            mid = start + 1
            end = mid + 1
    output = []
    for i in range(length_waypoints):
        if good_waypoints[i] == 1:
            output.append(waypoints[i,:])
    return np.array(output)

print(simplify_path(waypoints))