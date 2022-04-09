import networkx as nx
from pyvis import network as net
import time
from queue import PriorityQueue
from typing import Union, Tuple

class Graph:
    def __init__(self, fd):
        # Verifica a validade do descritor do arquivo
        if fd is None:
            raise ValueError("File descriptor is none")
        
        self.graph = nx.DiGraph()
        self.edges = {}
        self.lastNodeId = 0
        self.clients = {}
        self.cars = {}

        # Pula o cabeçalho do arquivo de entrada
        fd.readline()

        # Lê cada linha do arquivo de entrada
        for line in fd:
            # Separa os valores de input em um lista
            values = line.strip().split()
            edge_id         = values[0]         # str
            node_orig_id    = values[1]         # str
            node_orig_x     = float(values[2])
            node_orig_y     = float(values[3])
            node_dest_id    = values[4]         # str
            node_dest_x     = float(values[5])
            node_dest_y     = float(values[6])
            distance        = float(values[7])
            speed           = float(values[8])
            '''
                values[0] = Aresta_n
                values[1] = v_origem
                values[2] = Loc_v_origem (X)
                values[3] = Loc_v_origem (Y)
                values[4] = v_destino
                values[5] = Loc_v_destino (X)
                values[6] = Loc_v_destino (Y)
                values[7] = Distância(km)
                values[8] = Velocidade(km/h)
            '''
            
            # Quantidade de elementos da linha está incorreto
            if len(values) != 9 :
                raise ValueError("Misformatted graph file")
            
            # Mantem registro do maior id de nó
            if self.lastNodeId < int(node_orig_id): self.lastNodeId = int(node_orig_id)
            if self.lastNodeId < int(node_dest_id): self.lastNodeId = int(node_dest_id)

            # Adiciona nos ao grafo. A multiplicacao por 50 é para melhorar a visualização
            self.graph.add_node(node_orig_id, x = node_orig_x*50, y = node_orig_y*50)
            self.graph.add_node(node_dest_id, x = node_dest_x*50, y = node_dest_y*50)
            
            wheight_time = (int(distance)/int(speed))*3600

            # Formata tempo em formato hh:mm
            time_string = time.strftime('%H:%M', time.gmtime(wheight_time))
            edge_title = f"ID = {edge_id}<br>"
            edge_title += f"Speed = {speed}km/h<br>"
            edge_title += f"Distance = {distance}km<br>"
            edge_title += f"Time = {time_string}"

            self.edges[edge_id] = (node_orig_id, node_dest_id)
            #str(int(distance)/int(speed))
            self.graph.add_edge(node_orig_id, node_dest_id, distance=distance, speed=speed, time=wheight_time, title=edge_title, id=edge_id)

    def genNewId(self):
        self.lastNodeId += 1
        return self.lastNodeId

    def addCar(self, position : Tuple[float, float], edge_id : str) -> str:
        """
        Input: position - tuple of x and y coordinates, id of which edge the car is currently on
        Output: Created car ID
        Behavior: Creates new car as an unconnected node in the graph
        """
        carId = str(self.genNewId())
        car_title = "Car<br>"
        car_title += f"ID = {carId}<br>"
        car_title += f"Position = {position}"

        self.cars[carId] = (position, edge_id)
        self.graph.add_node(
            carId,
            x = position[0]*50,
            y = position[1]*50,
            edge = edge_id,
            shape = "square",
            title = car_title,
            color = "orange"
        )
        return carId

    def addClient(self, position : Tuple[float, float], destination : Tuple[float, float]) -> str:
        """
        Input: position and destination - tuples of x and y coordinates
        Output: Created client's ID
        Behavior: Creates new client as an unconnected node in the graph
        """
        clientId = str(self.genNewId())
        edge_title = "Client<br>"
        edge_title += f"ID = {clientId}<br>"
        edge_title += f"Origem = {position}<br>"
        edge_title += f"Destino = {destination}"

        self.clients[clientId] = (position, destination)
        self.graph.add_node(
            clientId,
            x = position[0]*50,
            y = position[1]*50,
            orig = position,
            dest = destination,
            shape = "diamond",
            title = edge_title,
            color = "#F1919B"
        )
        return clientId

    # def addCar(position):

    def getSpeed(self, edge_id : str) -> Union[float, None]:
        """
        Input: edge's id
        Output: edge's speed
        Behavior: get edge's current speed. Returns None if edge doesn't exists
        """
        try:
            orig, dest = self.edges[edge_id]
            speed = self.graph.edges[orig, dest]["speed"]
        except KeyError:
            return None
        else:
            return speed

    def getDistance(self, edge_id : str) -> Union[float, None]:
        """
        Input: edge's id
        Output: edge's distance
        Behavior: get edge's current distance. Returns None if edge doesn't exists
        """
        try:
            orig, dest = self.edges[edge_id]
            distance = self.graph.edges[orig, dest]["distance"]
        except KeyError:
            return None
        else:
            return distance
            
    def getTime(self, edge_id : str) -> Union[float, None]:
        """
        Input: edge's id
        Output: edge's time
        Behavior: get edge's current time. Returns None if edge doesn't exists
        """
        try:
            orig, dest = self.edges[edge_id]
            time = self.graph.edges[orig, dest]["time"]
        except KeyError:
            return None
        else:
            return time

    def updateTitle(self, edge_id : str) -> None:
        """
        Input: edge's id
        Behavior: updates egde's title based on it's current attributes
        """
        orig, dest = self.edges[edge_id]
        edge_dict = self.graph.edges[orig, dest]
        wheight_time = edge_dict['time']
        new_title = f"ID = {edge_dict['id']}<br>"
        new_title += f"Speed = {edge_dict['speed']}km/h<br>"
        new_title += f"Distance = {edge_dict['distance']}km<br>"
        new_title += f"Time = {time.strftime('%H:%M', time.gmtime(wheight_time))}"
        self.graph.edges[orig, dest]["title"] = new_title

    def changeSpeed(self, edge_id : str, speed : float) -> bool:
        """
        Input: edge id, new speed
        Output: True if sucessful. False if edge is not found.
        Behavior: changes speed of specified edge
        """
        try:
            orig, dest = self.edges[edge_id]
            distance = self.graph.edges[orig, dest]["distance"]
            wheight_time = (int(distance)/int(speed))*3600
            self.graph.edges[orig, dest]["speed"] = speed
            self.graph.edges[orig, dest]["time"] = wheight_time
        except KeyError:
            return False
        else:
            self.updateTitle(edge_id)
            return True

    def showGraph(self):
        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        graph_plot.from_nx(self.graph)
        graph_plot.toggle_physics(False)
        graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')

    def dijkstra(self, origin, destiny = None):
        visited = {}
        distances = {}
        pq = PriorityQueue()
      
        for node in self.graph.nodes:
            visited[node] = False
            distances[node] = float('inf')

        distances[origin] = 0

        pq.put((distances[origin], origin))
        while not pq.empty():
            _, current = pq.get()
            if not visited[current]:
                visited[current] = True
                for neighbor in list(self.graph.neighbors(current)):
                    cost = self.graph.edges[current, neighbor]["time"]
                    if distances[current] + cost <= distances[neighbor]:
                        distances[neighbor] = distances[current] + cost
                        pq.put((distances[neighbor], neighbor))
        
        if destiny is None: return distances
        return distances[destiny]
