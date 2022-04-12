from operator import itemgetter
import networkx as nx
from pyvis import network as net
import time
from queue import PriorityQueue
from typing import Union, Tuple
import numpy as np
import math

class Graph:
    def __init__(self, fd):
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

    def addPontoDeEmbarque(self, position : Tuple[float, float]) -> str:
        """
        Input: position and destination - tuples of x and y coordinates
        Output: Created client's ID
        Behavior: Creates new client as an unconnected node in the graph
        """
        starId = str(self.genNewId())
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

        # graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        # graph_plot.from_nx(self.Rgraph)
        # graph_plot.toggle_physics(False)

        # graph_plot.toggle_drag_nodes(False)
        # graph_plot.show('graph2.html')

    def calc_line_equation(self, node1, node2):
        a = node2[1] - node1[1]
        b = node1[0] - node2[0]
        c = a*node1[0] + b*node1[1]
        return a, b, c

    def ortogonal(self, client, m):
        a = -m
        b = 1
        c = client[1] - (m*client[0])
        return a, b, c

    def distance(self, node1, node2):
        return math.sqrt(((node2[0] - node1[0]) ** 2) + ((node2[1] - node1[1]) ** 2))

    def intersection_point(self, node1, node2, client):
        normal_eq = self.calc_line_equation(node1, node2)
        if normal_eq[0] == 0:
            return client[0], normal_eq[2]/normal_eq[1]
        ortogonal_eq = self.ortogonal(client, normal_eq[1]/normal_eq[0])
        det = (normal_eq[0] * ortogonal_eq[1]) - (ortogonal_eq[0] * normal_eq[1])
        x = ((ortogonal_eq[1] * normal_eq[2]) - (normal_eq[1] * ortogonal_eq[2]))/det
        y = ((normal_eq[0] * ortogonal_eq[2]) - (ortogonal_eq[0] * normal_eq[2]))/det
        return x, y

    def client_suffer(self, client_id):
        x, y = self.graph.nodes[client_id]["orig"]
        chosen_one = (float("inf"), None, -1)
        for edge in self.graph.edges:
            node1_id, node2_id = edge
            node1 = self.graph.nodes[node1_id]["orig"]
            node2 = self.graph.nodes[node2_id]["orig"]
            
            intersection = self.intersection_point(node1, node2, (x, y))
            dist_node_1 = self.distance(intersection, node1)
            dist_node_2 = self.distance(intersection, node2)

            loopDistance = None

            if not math.isclose(dist_node_1 + dist_node_2, self.distance(node1, node2)):
                if dist_node_1 < dist_node_2:
                    loopDistance = (self.distance((x, y), node1), node1, edge)
                else:
                    loopDistance = (self.distance((x, y), node2), node2, edge)

            else:
                loopDistance = (self.distance((x, y), intersection), intersection, edge)

            if loopDistance[0] < chosen_one[0]: chosen_one = loopDistance

        return {"clientPosition": chosen_one[1], "edge": chosen_one[2]}

        

    def getShortestPath(self, origin, destination):
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
            
            while current != origin:
                path.insert(0, current)
                current = prev[current]
            
            path.insert(0, current)
        
        return path, distance

    def dijkstra(self, origin, destination = None):
        """
            Entrada:
                self: próprio grafo
                origin: origem da busca, é um inteiro como string. Ex: '2', '3'
                destination: destino da busca, segue o mesmo padrão de origin

            Saída:
                Tupla: (distancia, anteriores)
                    distancia: a distância deste menor caminho encontrado
                    anteriores: dicionário em que os valores são associados às chaves que eles desembocam
                        Ex: '2': '1' significa que o vértice 2 é acessível através do vértice 1 (1 -> 2)
        """
        visited = {}
        distances = {}
        pq = PriorityQueue()
        prev = {}

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
                        
                        prev[neighbor] = current
        
        if destination is None: return distances
        return distances, prev  

    def yenkShortestPaths(self, origin, destination, k=5):
        """
            Entrada:
                self: próprio grafo
                origin: origem da busca, é um inteiro como string. Ex: '2', '3'
                destination: destino da busca, segue o mesmo padrão de origin

            Saída:
                Tupla: (distancia, anteriores)
                    distancia: a distância deste menor caminho encontrado
                    anteriores: dicionário em que os valores são associados às chaves que eles desembocam
                        Ex: '2': '1' significa que o vértice 2 é acessível através do vértice 1 (1 -> 2)
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

if __name__ == "__main__":
    fd = open("input3.txt", "r")
    g = Graph(fd)
    clientId = g.addClient((7, 10), (7, 7))
    coco = g.client_suffer(clientId)
    g.addPontoDeEmbarque(coco["clientPosition"])
    print(coco)
    g.showGraph()
