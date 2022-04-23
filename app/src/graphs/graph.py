from operator import itemgetter
import networkx as nx
from pyvis import network as net
import time
from queue import PriorityQueue
from typing import Union, Tuple, Dict, List
import numpy as np
import math
from os import sep

#ICONS_PATH = "app" + sep + "static" + sep
ICONS_PATH = ".." + sep + ".." + sep + "static" + sep

class Graph:
    def __init__(self, fd):
        """Constroi objeto grafo a partir de dado arquivo

        Args:
            fd (file_descriptor): descritor do arquivo que servirá para a montagem do grafo

        Raises:
            ValueError: "File descriptor is none"
            ValueError: "Misformatted graph file"
        """

        # Verifica a validade do descritor do arquivo
        if fd is None:
            raise ValueError("File descriptor is none")

        # lista de cores padrao. [0] eh a cor de asfalto (preto)
        self.colorList = ["#303030", "royalblue", "firebrick", "darkorange", "seagreen", "darkcyan"]
        self.zoomOut = 85

        self.graph = nx.DiGraph()
        self.Rgraph = nx.DiGraph()
        self.edges = {}
        self.lastNodeId = 0
        self.clients = {}
        self.cars = {}
        self.nodes = {}
        self.waitTime = 0
        self.travelTime = 0
        self.totalTime = 0
        self.numberOfTravels = 0

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
            edge_orig_title = f"ID = {node_orig_id}<br>"
            edge_orig_title += f"Position = ({node_orig_x}, {node_orig_y})"
            edge_dest_title = f"ID = {node_dest_id}<br>"
            edge_dest_title += f"Position = ({node_dest_x}, {node_dest_y})"

            self.nodes[(node_orig_x, node_orig_y)] = node_orig_id
            self.nodes[(node_dest_x, node_dest_y)] = node_dest_id
            self.graph.add_node(node_orig_id, x = node_orig_x*self.zoomOut, y = node_orig_y*self.zoomOut, orig = (node_orig_x, node_orig_y), color=self.colorList[0], title=edge_orig_title)
            self.graph.add_node(node_dest_id, x = node_dest_x*self.zoomOut, y = node_dest_y*self.zoomOut, orig = (node_dest_x, node_dest_y), color=self.colorList[0], title=edge_dest_title)
            self.Rgraph.add_node(node_orig_id, x = node_orig_x*self.zoomOut, y = node_orig_y*self.zoomOut, orig = (node_orig_x, node_orig_y), color=self.colorList[0], title=edge_orig_title)
            self.Rgraph.add_node(node_dest_id, x = node_dest_x*self.zoomOut, y = node_dest_y*self.zoomOut, orig = (node_dest_x, node_dest_y), color=self.colorList[0], title=edge_dest_title)
            
            wheight_time = (int(distance)/int(speed))*3600

            # Formata tempo em formato hh:mm
            time_string = time.strftime('%H:%M', time.gmtime(wheight_time))
            edge_title = f"ID = {edge_id}<br>"
            edge_title += f"Speed = {speed}km/h<br>"
            edge_title += f"Distance = {distance}km<br>"
            edge_title += f"Time = {time_string}"

            self.edges[edge_id] = (node_orig_id, node_dest_id)
            #str(int(distance)/int(speed))
            self.graph.add_edge(node_orig_id, node_dest_id, distance=distance, speed=speed, time=wheight_time, title=edge_title, id=edge_id, color=self.colorList[0], width=1)
            self.Rgraph.add_edge(node_dest_id, node_orig_id, distance=distance, speed=speed, time=wheight_time, title=edge_title, id=edge_id, color=self.colorList[0], width=1)

    def __genNewId(self):
        self.lastNodeId += 1
        return self.lastNodeId

    def clientToNode(self, position: Tuple[float, float]) -> Tuple[Tuple[str, float, float], Tuple[str, float, float], Tuple[float, float]]:
        """ Calcula o offset de distância e tempo de um cliente aos vértices mais próximos

        Args:
            position (Tuple[float, float]): posição do cliente

        Returns:
            Tuple[Tuple[str, float, float], Tuple[str, float, float]]: ((id_da_resta_origem, offset_distancia, offset_tempo), (id_da_resta_destino, offset_distancia, offset_tempo), (approx_x, approx_y))
        """
        dictionary = self.roadApprox(position)
        approx_x, approx_y = dictionary['position']
        id_node1, id_node2 = dictionary['edge']
                
        if (approx_x, approx_y) == self.graph.nodes[id_node1]['orig']:
            return ((id_node1, 0.0, 0.0), (id_node1, 0.0, 0.0), (approx_x, approx_y))

        elif (approx_x, approx_y) == self.graph.nodes[id_node2]['orig']:
            return ((id_node2, 0.0, 0.0), (id_node2, 0.0, 0.0), (approx_x, approx_y))

        else:
            node1_p = self.graph.nodes[id_node1]['orig']
            node2_p = self.graph.nodes[id_node2]['orig']

            distance_node = self.__distance(node1_p, (approx_x, approx_y))
            distance_total = self.__distance(node1_p, node2_p)

            relative = (distance_node/distance_total)

            real_time = self.graph.edges[id_node1, id_node2]['time']
            real_distance = self.graph.edges[id_node1, id_node2]['distance']

            dist_offset_orig = relative * real_distance
            time_offset_orig = relative * real_time
            dist_offset_dest = (1 - relative) * real_distance
            time_offset_dest = (1 - relative) * real_time

            return ((id_node1, dist_offset_orig, time_offset_orig), (id_node2, dist_offset_dest, time_offset_dest), (approx_x, approx_y))

    def carToNode(self, position : Tuple[float, float], edge_id : str) -> Tuple[str, float, float]:
        """ Aproxima um carro a um vértice e calcula a distância entre a posição original do carro e o vértice para o qual ele foi aproximado.

        Args:
            car_id (str): id do carro que será aproximado

        Returns:
            (str, float, float): tripla cuja primeira posição corresponde ao id do nó para o qual o carro foi aproximado, a segunda é a distância (em kilometros) entre este nó e a posição original do carro e a terceira o tempo gasto para percorrer esta distância até o nó.
        """
        # for x in self.cars:
        #     print(x)
        
        # car_edge eh a aresta em que o carro esta
        car_edge = self.edges[edge_id]
        nodeObj1 = self.graph.nodes[car_edge[0]]["orig"]
        nodeObj2 = self.graph.nodes[car_edge[1]]["orig"]
        distance_node = self.__distance(nodeObj2, position)
        distance_total = self.__distance(nodeObj2, nodeObj1)

        relative = (distance_node/distance_total)

        # retorna o próprio vértice com offsets 0 caso o carro esteja em cima de um vértice
        if self.__distance(nodeObj1, position) == 0:
            return (car_edge[0], 0, 0)

        return (car_edge[1], self.graph.edges[car_edge[0], car_edge[1]]["distance"] * relative, self.graph.edges[car_edge[0], car_edge[1]]["time"] * relative)

    def addCar(self, position : Tuple[float, float], edge_id : str) -> str:
        """
        Input: position - tuple of x and y coordinates, id of which edge the car is currently on
        Output: Created car ID
        Behavior: Creates new car as an unconnected node in the graph
        """
        carId = str(self.__genNewId())

        # print(carId)
        aprrox_node, dist_offset, time_offset = self.carToNode(position, edge_id)

        node1_id, node2_id = self.edges[edge_id]
        node1 = self.graph.nodes[node1_id]["orig"]
        node2 = self.graph.nodes[node2_id]["orig"]
        
        intersection = self.__intersectionPoint(node1, node2, position)
        dist_node_1 = self.__distance(intersection, node1)
        dist_node_2 = self.__distance(intersection, node2)

        if not math.isclose(dist_node_1 + dist_node_2, self.__distance(node1, node2)):
            if dist_node_1 < dist_node_2:
                position = node1
            else:
                position = node2

        else:
            position = intersection

        car_title = "Car<br>"
        car_title += f"ID = {carId}<br>"
        car_title += f"Position = {self.__formatPosition(position)}"

        #self.cars[carId] = (position, edge_id, aprrox_node, dist_offset, time_offset)
        self.nodes[position] = carId
        self.cars[carId] = {'position': position , 'edge_id': edge_id , 'approx_node': aprrox_node , 
        'dist_offset': dist_offset , 'time_offset': time_offset}
        # self.cars[carId] = {'position': position , 'edge_id': edge_id}

        self.graph.add_node(
            carId,
            x = position[0]*self.zoomOut,
            y = position[1]*self.zoomOut,
            orig = position,
            edge = edge_id,
            shape = "image",
            size = 25,
            image = ICONS_PATH + 'car2.svg',
            title = car_title,
            color = "orange"
        )
        return carId

    def removeCar(self, carId):
        """Remove cars especificado pelo ID

        Args:
            carId (str): id do carro a ser removido
        """
        self.graph.remove_node(carId)
        del self.cars[carId]

    def addClient(self, position : Tuple[float, float], destination : Tuple[float, float]) -> str:
        """
        Input: position and destination - tuples of x and y coordinates
        Output: Created client's ID
        Behavior: Creates new client as an unconnected node in the graph
        """
        clientId = str(self.__genNewId())
        edge_title = "Client<br>"
        edge_title += f"ID = {clientId}<br>"
        edge_title += f"Origem = {position}<br>"
        edge_title += f"Destino = {destination}"

        client_offset = self.clientToNode(position)
        dest_offset = self.destToNode(destination)
        
        approx_node_prev, dist_offset_prev, time_offset_prev = client_offset[0]
        approx_node_next, dist_offset_next, time_offset_next = client_offset[1]
        x_approx_orig, y_approx_orig = client_offset[2]
        approx_node_dest, time_offset_dest, (x_approx_dest, y_approx_dest) = dest_offset

        self.nodes[position] = clientId
        self.clients[clientId] = {'position': position, 'destination': destination,
        'approx_node_prev': approx_node_prev, 'dist_offset_prev': dist_offset_prev, 
        'time_offset_prev': time_offset_prev, 'approx_node_next': approx_node_next,
        'dist_offset_next': dist_offset_next, 'time_offset_next': time_offset_next,
        'approx_node_dest': approx_node_dest, 'time_offset_dest': time_offset_dest,
        'approx_position_dest': (x_approx_dest, y_approx_dest), 'approx_position_orig': (x_approx_orig, y_approx_orig)}

        self.graph.add_node(
            clientId,
            x = position[0]*self.zoomOut,
            y = position[1]*self.zoomOut,
            orig = position,
            dest = destination,
            shape = "image",
            image = ICONS_PATH + "person.svg",
            size = 25,
            title = edge_title,
            color = "#F1919B"
        )
        self.graph.add_node(
            "-" + clientId,
            x = destination[0]*self.zoomOut,
            y = destination[1]*self.zoomOut,
            orig = destination,
            shape = "image",
            size=15,
            image = ICONS_PATH + "location_mark.svg",
            title = f"Destino do cliente {clientId}<br>Posição: {destination}",
            color = "#gold"
        )
        return clientId

    def removeClient(self, clientId : str) -> None:
        """Remove cliente especificado pelo ID

        Args:
            clientId (str): id do cliente a ser removido
        """
        self.graph.remove_node(clientId)
        self.graph.remove_node("-" + clientId)
        del self.clients[clientId]

    def drawClientOnRoad(self, position : Tuple[float, float]) -> str:       
        """
        Input: position and destination - tuples of x and y coordinates
        Output: Created client's ID
        Behavior: Creates new client as an unconnected node in the graph
        """
        starId = str(self.__genNewId())
        # print(starId)
        self.graph.add_node(
            starId,
            x = position[0]*self.zoomOut,
            y = position[1]*self.zoomOut,
            orig = position,
            shape = "star",
            color = "yellow"
        )

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

    def __updateTitle(self, edge_id : str) -> None:
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
            self.Rgraph.edges[orig, dest]["speed"] = speed
            self.graph.edges[orig, dest]["time"] = wheight_time
            self.Rgraph.edges[orig, dest]["time"] = wheight_time

        except KeyError:
            return False
        else:
            self.__updateTitle(edge_id)
            return True

    def changeEdgeColor(self, edge_id : str, color : str = "#303030"):
        """
        Input: edge id, new color (defalt to asphalt color)
        Output: True if sucessful. False if edge is not found.
        Behavior: changes color of specified edge
        """
        try:
            orig, dest = self.edges[edge_id]
            self.graph.edges[orig, dest]["color"] = color
        except KeyError:
            return False
        else:
            return True

    def paintPath(self, path : List[int], color : str = "#303030") -> None:
        """Pinta dado caminho a partir da cor especificada

        Args:
            path (List[int]): lista de nos interligados que fecham um caminho
            color (str, optional): cor desejada. Defaults to "#303030".

        Raises:
            KeyError: Os vertices passados nao fecham um caminho
        """
        for i in range(len(path)-1):
            try:
                self.graph.edges[path[i], path[i+1]]["color"] = color            
                self.graph.edges[path[i], path[i+1]]["width"] = 3            
            except KeyError:
                print("Os vertices passados nao fecham um caminho")
                raise KeyError

    def resetColors(self) -> None:
        """Set all edges to asphalt color"""
        for orig, dest in self.graph.edges:
            self.graph.edges[orig, dest]["color"] = self.colorList[0]
            self.graph.edges[orig, dest]["width"] = 1

    def __formatPosition(self, position : Tuple[float, float]) -> Tuple[str, str]:
        return ('{:.2f}'.format(position[0]), '{:.2f}'.format(position[1]))

    def showGraphRoute(self, path : List, approx_orig : Tuple[float, float], approx_dest : Tuple[float, float], approx_interm : Tuple[float, float] = None):
        """Mostra o grafo
        Args:
            reverse (bool, optional): True se o objetivo é mostrar o grafo reverso. Padrão é Falso.
        """

        color = self.colorList[5]
        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True, font_color="#10000000")
        self.paintPath(path, color)
        graph_plot.from_nx(self.graph)

        new_id = self.__genNewId()
        new_id2 = self.__genNewId()

        # todo checar se tem um no na posicao. se tiver, nao sobreescreve e salva o id dele
        if path == []:
            car_id = self.nodes[approx_orig]
            path.append(car_id)
            graph_plot.add_node(
                str(new_id), 
                x=approx_dest[0]*self.zoomOut, y=approx_dest[1]*self.zoomOut, 
                title=f"Ponto de embarque do cliente<br>Posição: {self.__formatPosition(approx_dest)}",
                color="#32BA9F",
                shape="image",
                size=20,
                image=ICONS_PATH + "target_destination.svg"
            )
            path.append(new_id)
            graph_plot.add_edge(
                car_id,
                str(new_id),
                # title="qualquer coisa aqui",
                color=color,
                width=3,
            )
        elif approx_interm == None:
            # caso cliente ate destino
            if approx_orig not in self.nodes:
                graph_plot.add_node(
                    str(new_id), 
                    x=approx_orig[0]*self.zoomOut, y=approx_orig[1]*self.zoomOut, 
                    title=f"Ponto de embarque do cliente<br>Posição: {self.__formatPosition(approx_orig)}",
                    color="#32BA9F",
                    shape="image",
                    size=20,
                    image=ICONS_PATH + "target_destination.svg"
                )
                graph_plot.add_node(
                    str(new_id2), 
                    x=approx_dest[0]*self.zoomOut, y=approx_dest[1]*self.zoomOut, 
                    title=f"Ponto de desembarque do cliente<br>Posição: {self.__formatPosition(approx_dest)}",
                    color="#32BA9F",
                    shape="image",
                    size=20,
                    image=ICONS_PATH + "target.svg"
                )
                graph_plot.add_edge(
                    str(new_id),
                    path[0],
                    # title="qualquer coisa aqui",
                    color=color,
                    width=3
                )
                graph_plot.add_edge(
                    path[-1],
                    str(new_id2),
                    # title="qualquer coisa aqui",
                    color=color,
                    width=3
                )
                path.insert(0, new_id)
                path.append(new_id2)
            # caso carro ate cliente
            else:
                graph_plot.add_node(
                    str(new_id), 
                    x=approx_dest[0]*self.zoomOut, y=approx_dest[1]*self.zoomOut, 
                    title=f"Ponto de embarque do cliente<br>Posição: {self.__formatPosition(approx_dest)}",
                    color="#32BA9F",
                    shape="image",
                    size=20,
                    image=ICONS_PATH + "target_destination.svg"
                )
                car_id = self.nodes[approx_orig]
                graph_plot.add_edge(
                    car_id,
                    path[0], 
                    # title="qualquer coisa aqui",
                    color=color,
                    width=3
                )
                path.insert(0, car_id)
                graph_plot.add_edge(
                    path[-1],
                    str(new_id), 
                    # title="qualquer coisa aqui",
                    color=color,
                    width=3
                )
                path.append(str(new_id))
        else:
            graph_plot.add_node(
                str(new_id), 
                x=approx_interm[0]*self.zoomOut, y=approx_interm[1]*self.zoomOut, 
                title=f"Ponto de embarque do cliente<br>Posição: {self.__formatPosition(approx_interm)}",
                color="#32BA9F",
                shape="image",
                label=None,
                image=ICONS_PATH + "target_destination.svg",
                size=20
            )
            graph_plot.add_node(
                str(new_id2), 
                x=approx_dest[0]*self.zoomOut, y=approx_dest[1]*self.zoomOut, 
                title=f"Ponto de desembarque do cliente<br>Posição: {self.__formatPosition(approx_dest)}",
                color="#32BA9F",
                shape="image",
                image=ICONS_PATH + "target.svg",
                size=20
            )
            graph_plot.add_edge(
                path[-1],
                str(new_id2), 
                # title="qualquer coisa aqui",
                color=color,
                width=3
            )
            path.append(str(new_id2))

            car_id = self.nodes[approx_orig]
            graph_plot.add_edge(
                car_id,
                path[0], 
                # title="qualquer coisa aqui",
                color=color,
                width=3
            )
            path.insert(0, car_id)

        graph_plot.toggle_physics(False)
        graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')
        # graph_plot.save_graph('app/static/graph.html')

    def showGraph(self, reverse = False):
        """Mostra o grafo
        Args:
            reverse (bool, optional): True se o objetivo é mostrar o grafo reverso. Padrão é Falso.
        """
        graph = self.graph

        if reverse:
            graph = self.Rgraph

        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True, font_color="#10000000")
        graph_plot.from_nx(graph)
        graph_plot.toggle_physics(False)

        graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')
        # graph_plot.save_graph('app/static/graph.html')

    def __calcLineEquation(self, node1 : Tuple[float, float], node2 : Tuple[float, float]) -> Tuple[float, float, float] :
        """ calculates line coefficients given two points coordinates

        Args:
            node1 ((float, float)): node coordinates x and y
            node2 ((float, float)): node coordinates x and y

        Returns:
            Tuple[float, float, float]: a, b, c coefficients (from ax + by = c)
        """

        a = node2[1] - node1[1]
        b = node1[0] - node2[0]
        c = a*node1[0] + b*node1[1]
        return a, b, c

    def __orthogonal(self, node : Tuple[float, float], m: float) -> Tuple[float, float, float]:
        """ calculates orthogonal from a given point coordinates x and y

        Args:
            node (Tuple[float, float]): (x, y) coordinates
            m (float): angular coefficient

        Returns:
            Tuple[float, float, float]: a, b, c coefficients (from ax + by = c)
        """
        a = -m
        b = 1
        c = node[1] - (m*node[0])
        return a, b, c

    def __distance(self, node1: Tuple[float, float], node2: Tuple[float, float]) -> float:
        """ calculates distance between two points coordinates

        Args:
            node1 (Tuple[float, float]): (x, y) coordinates
            node2 (Tuple[float, float]): (x, y) coordinates

        Returns:
            float: distance between two points coordinatess
        """

        return math.sqrt(((node2[0] - node1[0]) ** 2) + ((node2[1] - node1[1]) ** 2))

    def __intersectionPoint(self, node1 : Tuple[float, float], node2 : Tuple[float, float], node_to_edge : Tuple[float, float]) -> Tuple[float, float]:
        """ find intersection point between a pair of nodes and a given node 

        Args:
            node1 (Tuple[float, float]): (x, y) coordinates
            node2 (Tuple[float, float]): (x, y) coordinates
            node_to_edge (Tuple[float, float]): (x, y) coordinates

        Returns:
            Tuple[float, float]: intersection point coordinates
        """

        normal_eq = self.__calcLineEquation(node1, node2)
        if normal_eq[0] == 0:
            return node_to_edge[0], normal_eq[2]/normal_eq[1]
        orthogonal_eq = self.__orthogonal(node_to_edge, normal_eq[1]/normal_eq[0])
        det = (normal_eq[0] * orthogonal_eq[1]) - (orthogonal_eq[0] * normal_eq[1])
        x = ((orthogonal_eq[1] * normal_eq[2]) - (normal_eq[1] * orthogonal_eq[2]))/det
        y = ((normal_eq[0] * orthogonal_eq[2]) - (orthogonal_eq[0] * normal_eq[2]))/det
        return x, y

    def destToNode(self, position : Tuple[float, float]) -> Tuple[str, float, Tuple[float, float]]:
        """Aproxima o destino ao grafo

        Args:
            position (Tuple[float, float]): posição x, y do no

        Returns:
            Tuple[str, float]: tupla (id do nó ao qual o destino foi aproximado; offset do peso; (x, y) aproximados)
        """
        dictionary = self.roadApprox(position)
        edge_id = dictionary['edge']

         # dest_edge eh a aresta em que o carro esta
        aux = self.graph.edges[edge_id[0], edge_id[1]]['id']
        dest_edge = self.edges[aux]

        nodeObj1 = self.graph.nodes[dest_edge[0]]["orig"]
        nodeObj2 = self.graph.nodes[dest_edge[1]]["orig"]
        distance_node = self.__distance(nodeObj1, position)
        distance_total = self.__distance(nodeObj2, nodeObj1)

        relative = (distance_node/distance_total)

        # retorna o próprio vértice com offsets 0 caso o carro esteja em cima de um vértice
        if self.__distance(nodeObj2, position) == 0:
            return (dest_edge[1], 0)

        return (dest_edge[0], self.graph.edges[dest_edge[0], dest_edge[1]]["time"] * relative, dictionary["position"])

    def roadApprox(self, position: Tuple[float, float]) -> Dict[Tuple[float, float], str]:
        """ finds nearest edge (or node) of a unconnected node

        Args:
            position (Tuple[float, float]): unconnected node position

        Returns:
            Dict[Tuple[float, float], Tuple[str, str]]: x, y coordinates of approximated point, edge id
        """

        x, y = position
        chosen_one = (float("inf"), None, -1)
        for edge in self.graph.edges:
            node1_id, node2_id = edge
            node1 = self.graph.nodes[node1_id]["orig"]
            node2 = self.graph.nodes[node2_id]["orig"]
            
            intersection = self.__intersectionPoint(node1, node2, (x, y))
            dist_node_1 = self.__distance(intersection, node1)
            dist_node_2 = self.__distance(intersection, node2)

            loopDistance = None

            if not math.isclose(dist_node_1 + dist_node_2, self.__distance(node1, node2)):
                if dist_node_1 < dist_node_2:
                    loopDistance = (self.__distance((x, y), node1), node1, edge)
                else:
                    loopDistance = (self.__distance((x, y), node2), node2, edge)

            else:
                loopDistance = (self.__distance((x, y), intersection), intersection, edge)

            if loopDistance[0] < chosen_one[0]: chosen_one = loopDistance

        return {"position": chosen_one[1], "edge": chosen_one[2]}

    def getShortestPath(self, origin : str, destination : str) -> Tuple[List[str], float]:
        """
            Entrada:
                self: próprio grafo
                origin: origem da busca, é um inteiro como string. Ex: '2', '3'
                destination: destino da busca, segue o mesmo padrão de origin

            Saída:
                Tupla: (caminho, distancia)
                    caminho: menor caminho encontrado entre a origem e destino
                    distancia: a distância deste menor caminho encontrado
        """
        distance, prev = self.dijkstra(origin, destination)
        path = []

        if distance[destination] < float('inf'):
            path.insert(0, destination)
            current = prev[destination]
            
            if current != origin:
                while current != origin:
                    path.insert(0, current)
                    current = prev[current]

                path.insert(0, current)
        
        return path, distance

    def dijkstra(self, origin : str, destination : str = None, metric: str = "time", reverse = False):
        """
            Entrada:
                self (graph): próprio grafo
                origin (string): origem da busca, é um inteiro como string. Ex: '2', '3'
                destination (string): destino da busca, segue o mesmo padrão de origin
                metric (string): "time" para calcular considerando o tempo e "distance" a distância
                reverse (Bool): True se a busca deve ser feita no grafo reverso

            Saída:
                Se reverse == True
                    (commute_time, distances): tempo até a cada vértice, distância a cada vértice
                
                Se reverse == False e destination == None
                    (commute_time): tempo até cada vértice

                Do contário:
                Tupla: (distancia, anteriores)
                    distancia: a distância deste menor caminho encontrado
                    anteriores: dicionário em que os valores são associados às chaves que eles desembocam
                        Ex: '2': '1' significa que o vértice 2 é acessível através do vértice 1 (1 -> 2)
        """

        if reverse:
            graph = self.Rgraph
        else:
            graph = self.graph

        visited = {}
        route_values = {}
        pq = PriorityQueue()
        prev = {}

        for node in graph.nodes:
            visited[node] = False
            route_values[node] = float('inf')

        route_values[origin] = 0
        prev[origin] = origin

        pq.put((route_values[origin], origin))
        while not pq.empty():
            _, current = pq.get()
            if not visited[current]:
                visited[current] = True
                for neighbor in list(graph.neighbors(current)):
                    cost = graph.edges[current, neighbor][metric]
                    if route_values[current] + cost <= route_values[neighbor]:
                        route_values[neighbor] = route_values[current] + cost
                        pq.put((route_values[neighbor], neighbor))
                        
                        prev[neighbor] = current
        
        if reverse: return route_values
        if destination is None: return route_values
        return route_values, prev 

    # Recupera o valor de cada aresta
    def __getAllCosts(self):
        costs = {}
        for edge in self.graph.edges:
            value = self.graph.edges[edge[0], edge[1]]['time']
            costs[(edge[0], edge[1])] = value
        return costs

    # Calcula o custo de um caminho
    def __getCostFromPath(self, path, weights):
        cost = 0
        for i in range(0, len(path)-1):
            cost += weights[(path[i], path[i+1])]
        return cost

    def yenkShortestPaths(self, origin : str, destination : str, k=15) -> List[Dict]:
        """
            Entrada:
                self: próprio grafo
                origin: origem da busca, é um inteiro como string. Ex: '2', '3'
                destination: destino da busca, segue o mesmo padrão de origin

            Saída:
                Lista de dicionários, cujas chaves são 'cost' (custo do caminho) e 'path' (caminho) 
        """
        # Backup dos pesos das arestas
        weights = self.__getAllCosts()

        # obtém o menor caminho da origem ao destino, além das listas das distâncias a cada vértice
        path, distance = self.getShortestPath(origin, destination)

        # A = k menores caminhos
        A = [{'cost': distance[destination], 'path': path}]
        
        # B = canditados aos k menores caminhos
        B = []

        # garante que de fato existe um caminho. Caso não exista um caminho, retorna vazio
        if not A[0]['path']:
            return A

        # Busca os k melhores caminhos
        for i in range (1, k):

            # esse loop acontece uma quantidade de vezes igual ao número de vértices do último melhor caminho - 2 (é como se ignorasse o vértice inicial e final)
            for j in range (0, len(A[-1]['path']) - 1):
                
                # seleciona um vértice para refinamento
                spur_node = A[-1]['path'][j]

                # caminho do vértice inicial até o vértice de refinamento (incluso) (para análise)
                path_root = A[-1]['path'][:j+1]

                removed_edges = []

                # para cada caminho em A
                for path_k in A:
                    current_path = path_k['path']
                    
                    # se o tamanho do caminho atual for maior que j
                    # e o caminho para análise for igual ao caminho atual (até o vértice de refinamento -- incluso)
                    if len(current_path) - 1 > j and path_root == current_path[:j+1]:

                        # Remove todas as arestas que vao para os nos na raiz
                        for edge in self.graph.edges:
                            if edge[1] == current_path[j]:
                                weight = self.graph.edges[edge[0], edge[1]]['time']
                                removed_edges.append((edge[0], edge[1], weight))
                                self.graph.edges[edge[0], edge[1]]['time'] = float('inf')
                        
                        # backup do peso da aresta
                        weight = self.graph.edges[current_path[j], current_path[j+1]]['time']
                        
                        # caso o peso seja infinito apenas pulamos a iteração
                        if weight == float('inf'):
                            continue
                        
                        # muda temporariamente o peso (tempo de travessia) da aresta para infinito
                        removed_edges.append((current_path[j], current_path[j+1], weight))
                        self.graph.edges[current_path[j], current_path[j+1]]['time'] = float('inf')

                # obtém um novo caminho e vetor de distâncias, dessa vez indo do nó de refinamento até o destino 
                # (Busca alternativas desse nó de refinamento até o destino)
                path_spur, distance_spur = self.getShortestPath(spur_node, destination)

                # Se encontrou um novo caminho
                if path_spur:
                    # o caminho total passa a ser o caminho original concatenado com o novo caminho encontrado
                    path_total = path_root[:-1] + path_spur

                    # atualiza o vetor de distâncias de acordo com o novo caminho
                    dist_total = self.__getCostFromPath(path_root, weights) + distance_spur[destination]

                    # adiciona este caminho a lista de caminhos em potencial
                    potential_path = {'cost': dist_total, 'path':path_total}
                    # Verifica se o caminho ja nao esta em B
                    if not potential_path in B:
                        B.append(potential_path)

                # volta os pesos para as arestas 
                for edge in removed_edges:
                    self.graph.edges[edge[0], edge[1]]['time'] = edge[2]
            
            # Se a lista com os candidatos a menores caminhos possuir elementos
            if len(B):
                # ordenamos estes caminhos de acordo com o custo
                B = sorted(B, key=itemgetter('cost'))
                # adicionamos o melhor destes caminhos a lista com os k menores caminhos
                A.append(B[0])
                # e o removemos da lista de candidatos
                B.pop(0)
            else:
                break
        
        # A = k melhores caminhos
        return A    

    def carRoutes(self, client_id : str):
        """Determina a distância de um cliente para todos os carros livres

        Args:
            self (graph): o próprio grafo
            client_id (string): id do cliente que solicitou uma viagem

        Return:
            ({string: float}, {string: float}): tupla de dicionários:
                adjusted_distance: distância do carro até o cliente
                adjusted_times: tempo de viagem do carro até o cliente
        """
        # obtém a distância de um vértice a todos os outros no 
        client = self.edges[client_id]['approx_node_prev']
        distances = self.dijkstra(client, metric="distance", reverse=True)
        times = self.dijkstra(client, metric="time", reverse=True)
        
        adjusted_distances = {}
        adjusted_times = {}

        # itera por todos os carros
        for key in self.cars:
            # Distancia entre o carro e o cliente
            # Dois casos possiveis:
            # 1) Negativo -> O carro depois do cliente (usa o Dijkstra)
            # 2) Nao Negativo -> O carro antes do cliente (A distancia ate o cliente)
            betweenDistance = self.cars[key]['dist_offset'] - self.clients[key]['dist_offset_next']

            if self.cars[key]['approx_node'] == self.clients[client_id]['approx_node_next'] and betweenDistance >= 0:
                # distancia do carro ate o cliente
                adjusted_distances[key] = betweenDistance

                # tempo que o carro leva para ir ate o cliente
                adjusted_times[key] = self.cars[key]['time_offset'] - self.clients[key]['time_offset_next']
            else:
                # soma a distância do vértice para o qual o carro foi aproximado com o offset de ajuste do carro e do cliente
                adjusted_distances[key] = distances[self.cars[key]['approx_node']] + self.cars[key]['dist_offset'] + self.clients[client_id]['dist_offset_prev']
                
                # soma o tempo até o vértice para o qual o carro foi aproximado com o offset de ajuste
                adjusted_times[key] = times[self.cars[key]['approx_node']] + self.cars[key]['time_offset'] + self.clients[client_id]['time_offset_prev']

        return adjusted_distances, adjusted_times

    def clientRoutes(self, client : str) -> List[Dict]:
        """ retorna as 5 menores rotas do cliente até o destino
        Args:
            client (client): id do cliente

        Returns:
            Lista de dicionáros com os caminhos
        """

        # caminhos do cliente até o destino
        orig = self.clients[client]['approx_node_next']
        
        dest = self.clients[client]['approx_node_dest']

        betweenTime = self.clients[client]['time_offset_dest'] - self.clients[client]['time_offset_prev']

        if dest == self.clients[client]['approx_node_prev'] and betweenTime >= 0:
            paths = [{'path': [], 'cost': betweenTime}]
        else:
            paths = self.yenkShortestPaths(orig, dest)

            # adicona os offsets de tempo
            for path in paths:
                path['cost'] += self.clients[client]['time_offset_dest'] + self.clients[client]['time_offset_next']

        return paths

    def getCarRoute(self, client_id : str, car_id : str) -> Tuple[float, List[str]]:
        """ Calcula o trajeto mais rápido de um determinado carro até um cliente

        Args:
            client_id (str): id do cliente
            car_id (str): id do carro

        Returns:
            (custo, [caminho])
        """

        orig = self.cars[car_id]['approx_node']
        dest = self.clients[client_id]['approx_node_prev']
        
        betweenTime = self.cars[car_id]['time_offset'] - self.clients[client_id]['time_offset_next']

        if orig == self.clients[client_id]['approx_node_next'] and betweenTime >=0:
            cost = betweenTime
            path = []
        else:
            path, costs = self.getShortestPath(orig, dest)
            cost = costs[dest] + self.cars[car_id]['time_offset'] + self.clients[client_id]['time_offset_prev']

        return cost, path

    def getTotalPath(self, client_id : str, car_id : str, route : Dict) -> Tuple[float, List[str]]:
        """ Retorna o caminho total de um carro, passando pelo cliente e chegando até o destino.

        Returns:
            (custo, caminho)
        """
        
        car_client_cost, car_client_path = self.getCarRoute(client_id, car_id)

        if car_client_path != [] and car_client_path[-1] == route['path'][0]:
            total_path = car_client_path + route['path'][1:]
        else:
            total_path = car_client_path + route['path']

        
        total_cost = car_client_cost + route['cost']

        self.updateMetric(car_client_cost, route['cost'], total_cost)

        return total_cost, total_path

    def updateMetric(self, wait_time : float, travel_time : float, total_time : float) -> None:
        """Update das métricas "wait time" e "travel time"

        Simplesmente coloque o tempo de espera total que o último cliente sofreu, assim como o tempo de viagem.

        Args:
            wait_time (float): novo wait time
            travel_time (float): novo travel time
        """
        self.waitTime += wait_time
        self.travelTime += travel_time
        self.totalTime += total_time
        self.numberOfTravels += 1

    def getMetrics(self) -> Tuple[float, float]:
        """Retorna uma tupla com as métricas

        Returns:
            Tuple[float, float]: Média do wait time e média do travel time
        """
        if self.numberOfTravels == 0: return (0, 0)
        return (self.waitTime / self.numberOfTravels, self.travelTime / self.numberOfTravels)

if __name__ == "__main__":
    fd = open("input5.txt", "r")
    g = Graph(fd)
    clientId = g.addClient((0, 0), (6, 6))
    clientId2 = g.addClient((2, 4), (2.8, 4))
    # clientId2 = g.addClient((2, 4), (2, 4))
    carId = g.addCar((5.5, 3.5), "2")
    carId2 = g.addCar((8.5, 5), "4")
    carId3 = g.addCar((2.5, 4), "11")

    # routes = g.clientRoutes(clientId)
    # route2 = g.getCarRoute(clientId2, carId3)[1]

    # print(route2)

    # print("\n5 menores rotas:")
    # for route in routes:
    #     print(route)

    # print("\nCaminho Total")
    #path = g.getCarRoute(clientId, carId)
    #path2 = g.getCarRoute(clientId2, carId2)[1]

    print(g.yenkShortestPaths('6', '11'))

    # routes = g.clientRoutes(clientId2)[0]
    # path3 = routes["path"]
    # teste = g.getTotalPath(clientId2, carId3, routes)
    # print(teste)
    # g.showGraphRoute(teste[1], g.graph.nodes[carId3]["orig"], g.clients[clientId2]["approx_position_dest"], g.clients[clientId2]["approx_position_orig"])
    # g.showGraphRoute(path3, g.clients[clientId2]["approx_position_orig"], g.clients[clientId2]["approx_position_dest"])
    # g.showGraphRoute(route2, g.graph.nodes[carId3]["orig"], g.clients[clientId2]["approx_position_orig"])
    # routes = g.clientRoutes(clientId2)[0]
    # teste = g.getTotalPath(clientId2, carId2, routes)
    # g.showGraphRoute(teste[1], g.graph.nodes[carId2]["orig"], g.clients[clientId2]["approx_position_dest"], g.clients[clientId2]["approx_position_orig"])
    
    # g.paintPath(path, g.colorList[1])
    # g.showGraphRoute(path, g.graph.nodes[carId]["orig"], g.clients[clientId]["approx_position_dest"])
    # g.showGraphRoute(path, g.graph.nodes[carId]["orig"], g.clients[clientId]["approx_position_orig"])
    # time.sleep(3)
    # # print(g.dijkstra('1', reverse=True))
    # print(a)
    # print(b)
    # g.carRoutes(string, float1, float2)

  
    # yenk = g.yenkShortestPaths("15", "13")
    # print(yenk)
    # for i in range(1):
    #     g.resetColors()
    #     g.paintPath(yenk[i]["path"], g.colorList[i+1])
    #     g.showGraph()
        # time.sleep(10)

    # res = g.roadApprox(clientId)
    # g.drawClientOnRoad(res["clientPosition"])
    # print(res)
    # path = yenk["path"]
    # g.resetColors()
    # for i in range(1, 6):
    #     g.changeEdgeColor(str(i), g.colorList[i])
    # carId = g.addCar([1.5, 1.5], "1")
    # g.calc_offset(carId)
    # print(g.carToNode(carId))
