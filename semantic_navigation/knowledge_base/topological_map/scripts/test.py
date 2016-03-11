#!/usr/bin/env python
# license removed for brevity

import rospy

#graph = {'livingroom-01': ['door-01', 'door-02', 'door-03', 'door-04', 'door-05'],
#         'corridor-01': ['door-01'],
#         'bedroom-01': ['door-02'],
#	 'bedroom-02': ['door-03'],
#         'kitchen-01': ['door-04'],}

graph = {'livingroom-01': ['door-01', 'door-02', 'door-03', 'door-04', 'door-05'],
             'door-01': ['corridor-01'],
             'door-02': ['bedroom-01'],
             'door-03': ['bedroom-02'],
             'door-04': ['kitchen-01'],
             'door-05': ['kitchen-01', 'bedroom-02']}



def find_shortest_path(graph, start, end, path=[]):
    path = path + [start]
    
    if start == end:
        return path
    
    if not graph.has_key(start):
        return None
    shortest = None
    
    for node in graph[start]:
        if node not in path:
            newpath = find_shortest_path(graph, node, end, path)
            if newpath:
                if not shortest or len(newpath) < len(shortest):
                    shortest = newpath
    return shortest             


if __name__ == '__main__':    
    path = find_shortest_path(graph, 'livingroom-01', 'corridor-01')

    for value in path:
        print(value)
