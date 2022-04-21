from operator import itemgetter
import networkx as nx
from pyvis import network as net
import time
from queue import PriorityQueue
from typing import Union, Tuple, Dict, List
import numpy as np
import math

from sklearn import metrics
from sklearn.tree import DecisionTreeClassifier

class Graph:
    def __init__(self, fd):
        """_summary_

        Args:
            fd (file_descriptor): descritor do arquivo que servirá para a montagem do grafo

        Raises:
            ValueError: "File descriptor is none"
            ValueError: "Misformatted graph file"
        """

        # Verifica a validade do descritor do arquivo
        if fd is None:
            raise ValueError("File descriptor is none")
        
        self.graph = nx.DiGraph()
        self.Rgraph = nx.DiGraph()
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
            self.graph.add_node(node_orig_id, x = node_orig_x*50, y = node_orig_y*50, orig = (node_orig_x, node_orig_y))
            self.graph.add_node(node_dest_id, x = node_dest_x*50, y = node_dest_y*50, orig = (node_dest_x, node_dest_y))
            self.Rgraph.add_node(node_orig_id, x = node_orig_x*50, y = node_orig_y*50, orig = (node_orig_x, node_orig_y))
            self.Rgraph.add_node(node_dest_id, x = node_dest_x*50, y = node_dest_y*50, orig = (node_dest_x, node_dest_y))
            
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
            self.Rgraph.add_edge(node_dest_id, node_orig_id, distance=distance, speed=speed, time=wheight_time, title=edge_title, id=edge_id)

    def __genNewId(self):
        self.lastNodeId += 1
        return self.lastNodeId

    def client_to_node(self, position: Tuple[float, float]) -> Tuple[Tuple[str, float, float], Tuple[str, float, float]]:
        """ Calcula o offset de distância e tempo de um cliente aos vértices mais próximos

        Args:
            position (Tuple[float, float]): posição do cliente

        Returns:
            Tuple[Tuple[str, float, float], Tuple[str, float, float]]: ((id_da_resta_origem, offset_distancia, offset_tempo), (id_da_resta_destino, offset_distancia, offset_tempo))
        """
        dictionary = self.road_approx(position)
        approx_x, approx_y = dictionary['position']
        id_node1, id_node2 = dictionary['edge']
        
        
        if (approx_x, approx_y) == self.graph.nodes[id_node1]['orig']:
            return ((id_node1, 0.0, 0.0), (id_node1, 0.0, 0.0))

        elif (approx_x, approx_y) == self.graph.nodes[id_node2]['orig']:
            return ((id_node2, 0.0, 0.0), (id_node2, 0.0, 0.0))

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

            return ((id_node1, dist_offset_orig, time_offset_orig), (id_node2, dist_offset_dest, time_offset_dest))



    def car_to_node(self, position : Tuple[float, float], edge_id : str) -> Tuple[str, float, float]:
        """ Aproxima um carro a um vértice e calcula a distância entre a posição original do carro e o vértice para o qual ele foi aproximado.

        Args:
            car_id (str): id do carro que será aproximado

        Returns:
            (str, float, float): tripla cuja primeira posição corresponde ao id do nó para o qual o carro foi aproximado, a segunda é a distância (em kilometros) entre este nó e a posição original do carro e a terceira o tempo gasto para percorrer esta distância até o nó.
        """
        for x in self.cars:
            print(x)
        
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
        car_title = "Car<br>"
        car_title += f"ID = {carId}<br>"
        car_title += f"Position = {position}"

        # print(carId)
        aprrox_node, dist_offset, time_offset = self.car_to_node(position, edge_id)

        #self.cars[carId] = (position, edge_id, aprrox_node, dist_offset, time_offset)
        self.cars[carId] = {'position': position , 'edge_id': edge_id , 'approx_node': aprrox_node , 'dist_offset': dist_offset , 'time_offset': time_offset}
        # self.cars[carId] = {'position': position , 'edge_id': edge_id}

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
        clientId = str(self.__genNewId())
        edge_title = "Client<br>"
        edge_title += f"ID = {clientId}<br>"
        edge_title += f"Origem = {position}<br>"
        edge_title += f"Destino = {destination}"

        client_offset = self.client_to_node(position)
        dest_offset = self.dest_to_node(destination)
        
        approx_node_prev, dist_offset_prev, time_offset_prev = client_offset[0]
        approx_node_next, dist_offset_next, time_offset_next = client_offset[1]
        approx_node_dest, time_offset_dest = dest_offset

        self.clients[clientId] = {'position': position, 'destination': destination, 'approx_node_prev': approx_node_prev, 'dist_offset_prev': dist_offset_prev, 'time_offset_prev': time_offset_prev, 'approx_node_next': approx_node_next, 'dist_offset_next': dist_offset_next, 'time_offset_next': time_offset_next,
        'approx_node_dest': approx_node_dest, 'time_offset_dest': time_offset_dest}

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

    def drawClientOnRoad(self, position : Tuple[float, float]) -> str:       
        """
        Input: position and destination - tuples of x and y coordinates
        Output: Created client's ID
        Behavior: Creates new client as an unconnected node in the graph
        """
        starId = str(self.__genNewId())
        print(starId)
        self.graph.add_node(
            starId,
            x = position[0]*50,
            y = position[1]*50,
            orig = position,
            shape = "star",
            color = "yellow"
        )

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

    def get__Distance(self, edge_id : str) -> Union[float, None]:
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
            self.graph.edges[orig, dest]["time"] = wheight_time
        except KeyError:
            return False
        else:
            self.__updateTitle(edge_id)
            return True

    def showGraph(self, reverse = False):
        """Mostra o grafo
        Args:
            reverse (bool, optional): True se o objetivo é mostrar o grafo reverso. Padrão é Falso.
        """
        graph = self.graph

        if reverse:
            graph = self.Rgraph

        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        graph_plot.from_nx(graph)
        graph_plot.toggle_physics(False)

        #graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')

        # graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        # graph_plot.from_nx(self.Rgraph)
        # graph_plot.toggle_physics(False)

        # graph_plot.toggle_drag_nodes(False)
        # graph_plot.show('graph2.html')

    def __calc_line_equation(self, node1 : Tuple[float, float], node2 : Tuple[float, float]) -> Tuple[float, float, float] :
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

    def __intersection_point(self, node1 : Tuple[float, float], node2 : Tuple[float, float], node_to_edge : Tuple[float, float]) -> Tuple[float, float]:
        """ find intersection point between a pair of nodes and a given node 

        Args:
            node1 (Tuple[float, float]): (x, y) coordinates
            node2 (Tuple[float, float]): (x, y) coordinates
            node_to_edge (Tuple[float, float]): (x, y) coordinates

        Returns:
            Tuple[float, float]: intersection point coordinates
        """

        normal_eq = self.__calc_line_equation(node1, node2)
        if normal_eq[0] == 0:
            return node_to_edge[0], normal_eq[2]/normal_eq[1]
        orthogonal_eq = self.__orthogonal(node_to_edge, normal_eq[1]/normal_eq[0])
        det = (normal_eq[0] * orthogonal_eq[1]) - (orthogonal_eq[0] * normal_eq[1])
        x = ((orthogonal_eq[1] * normal_eq[2]) - (normal_eq[1] * orthogonal_eq[2]))/det
        y = ((normal_eq[0] * orthogonal_eq[2]) - (orthogonal_eq[0] * normal_eq[2]))/det
        return x, y

    def dest_to_node(self, position : Tuple[float, float]) -> Tuple[str, float]:

        dictionary = self.road_approx(position)
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

        return (dest_edge[0], self.graph.edges[dest_edge[0], dest_edge[1]]["time"] * relative)


    def road_approx(self, position: Tuple[float, float]) -> Dict[Tuple[float, float], str]:
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
            
            intersection = self.__intersection_point(node1, node2, (x, y))
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

    def yenkShortestPaths(self, origin : str, destination : str, k=5) -> List[Dict]:
        """
            Entrada:
                self: próprio grafo
                origin: origem da busca, é um inteiro como string. Ex: '2', '3'
                destination: destino da busca, segue o mesmo padrão de origin

            Saída:
                Lista de dicionários, cujas chaves são 'cost' (custo do caminho) e 'path' (caminho) 
        """
        
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
                    if len(current_path) > j and path_root == current_path[:j+1]:
                        
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
                    dist_total = distance[spur_node] + distance_spur[destination]

                    # adiciona este caminho a lista de caminhos em potencial
                    potential_path = {'cost': dist_total, 'path':path_total}
                    if potential_path not in B:
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

    def car_routes(self, client : str, dist_offset : float, time_offset : float):
        """Determina a distância de um cliente para todos os carros livres

        Args:
            self (graph): o próprio grafo
            client (string): origem da busca, é um inteiro como string. Ex: '2', '3' e representa o vértice para o qual o cliente foi aproximado.
            
            time_offset: tempo gasto do ponto na aresta para o qual o cliente foi aproximado até o vértice que será levado em conta no cálculo.

            dist_offset (float): distância do ponto na aresta para o qual o cliente foi aproximado até o vértice que será levado em conta na busca pelo menor caminho.

        Return:
            ({string: float}, {string: float}): tupla de dicionários:
                adjusted_distance: distância do carro até o cliente
                adjusted_times: tempo de viagem do carro até o cliente
        """
        # obtém a distância de um vértice a todos os outros no 
        distances = self.dijkstra(client, metric="distance", reverse=True)
        times = self.dijkstra(client, metric="time", reverse=True)
        
        adjusted_distances = {}
        adjusted_times = {}

        # itera por todos os carros
        for key in self.cars:
            # soma a distância do vértice para o qual o carro foi aproximado com o offset de ajuste do carro e do cliente
            # print("distancia do vertice que o carro foi aproximado", distances[self.cars[key]['approx_node']])
            # print("offset da distancia do carro", self.cars[key]['dist_offset'])
            # print("offset do cliente", dist_offset)
            
            adjusted_distances[key] = distances[self.cars[key]['approx_node']] + self.cars[key]['dist_offset'] + dist_offset
            
            # soma o tempo até o vértice para o qual o carro foi aproximado com o offset de ajuste
            adjusted_times[key] = times[self.cars[key]['approx_node']] + self.cars[key]['time_offset'] + time_offset

        return adjusted_distances, adjusted_times

    def client_routes(self, client : str) -> List[Dict]:
        """ retorna as 5 menores rotas do cliente até o destino
        Args:
            client (client): id do cliente

        Returns:
            Lista de dicionáros com os caminhos
        """

        # caminhos do cliente até o destino
        orig = self.clients[client]['approx_node_next']
        
        dest = self.clients[client]['approx_node_dest']

        paths = self.yenkShortestPaths(orig, dest)

        # adicona os offsets de tempo
        for path in paths:
            path['cost'] += self.clients[client]['time_offset_dest'] + self.clients[client]['time_offset_next']

        return paths

    def getCarRoute(self, client_id : str, car_id : str) -> Tuple[float, List[str]]:
        """ Calcula a distância de um determinado carro até um cliente

        Args:
            client_id (str): id do cliente
            car_id (str): id do carro

        Returns:
            (custo, [caminho])
        """

        orig = self.cars[car_id]['approx_node']
        dest = self.clients[client_id]['approx_node_prev']
        path, costs = self.getShortestPath(orig, dest)
        cost = costs[dest] + self.cars[car_id]['time_offset'] + self.clients[client_id]['time_offset_prev']

        return cost, path

    def getTotalPath(self, client_id : str, car_id : str, route : Dict) -> Tuple[float, List[str]]:
        """ Retorna o caminho total de um carro, passando pelo cliente e chegando até o destino.

        Returns:
            (custo, caminho)
        """
        
        car_client_cost, car_client_path = self.getCarRoute(client_id, car_id)

        if car_client_path[-1] == route['path'][0]:
            total_path = car_client_path + route['path'][1:]
        else:
            total_path = car_client_path + route['path']
        
        total_cost = car_client_cost + route['cost']

        return total_cost, total_path


        


if __name__ == "__main__":
    fd = open("C:\\Users\\Joaop\\OneDrive\\Documentos\\UnB\\PAA\\App\\flascars\\app\\input5.txt", "r")
    
    import time
    print(time.time())
    g = Graph(fd)
    print(time.time())

    
    # carId = g.addCar([2, 1], "7")
    # carId = g.addCar([3, 1], "7")
    # clientId = g.addClient((1.5, 1.5), (11, 9))

    clientId = g.addClient((0, 0), (6, 6))
    clientId2 = g.addClient((2, 4), (7, 3.5))
    carId = g.addCar((5.5, 3.5), "2")
    carId2 = g.addCar((8.5, 5), "4")


    routes = g.client_routes(clientId)

    print("\n5 menores rotas:")
    for route in routes:
        print(route)

    print("\nCaminho Total")
    print(g.getTotalPath(clientId, carId, routes[0]))
    
    g.showGraph()
    # print(g.dijkstra('1', reverse=True))
    # print(a)
    # print(b)
    # g.car_routes(string, float1, float2)

  
