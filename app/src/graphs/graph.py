import networkx as nx
from pyvis import network as net
import time
from queue import PriorityQueue

class Graph:
    def __init__(self, fd):
        # Verifica a validade do descritor do arquivo
        if fd is None:
            raise ValueError("File descriptor is none")
        
        self.graph = nx.DiGraph()
        
        # Pula o cabeçalho do arquivo de entrada
        fd.readline()

        # Lê cada linha do arquivo de entrada
        for line in fd:
            # Separa os valores de input em um lista
            values = line.strip().split()
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
            
            # Adiciona nos ao grafo. A multiplicacao por 50 é para melhorar a visualização
            self.graph.add_node(values[1], x = float(values[2])*50, y = float(values[3])*50)
            self.graph.add_node(values[4], x = float(values[5])*50, y = float(values[6])*50)
            
            wheight_time = (float(values[7])/float(values[8]))*3600

            # Formata tempo em formato hh:mm
            time_string = time.strftime('%H:%M', time.gmtime(wheight_time))
            edge_title = f"ID = {values[0]}<br>"
            edge_title += f"Speed = {values[8]}km/h<br>"
            edge_title += f"Distance = {values[7]}km<br>"
            edge_title += f"Time = {time_string}"
            
            #str(int(values[7])/int(values[8]))
            self.graph.add_edge(values[1], values[4], distance=values[7], speed=values[8], time=wheight_time, title=edge_title)

    def showGraph(self):
        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        graph_plot.from_nx(self.graph)
        graph_plot.toggle_physics(False)
        # graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')

    """
        Entrada:
            self: próprio grafo
            origin: origem da busca, é um inteiro como string. Ex: '2', '3'
            destiny: destino da busca, segue o mesmo padrão de origin

        Saída:
            Tupla: (caminho, distancia)
                caminho: menor caminho encontrado entre a origem e destino
                distancia: a distância deste menor caminho encontrado
    """
    def getShortestPath(self, origin, destiny):
        distance, prev = self.dijkstra(origin, destiny)
        path = []

        if distance < float('inf'):
            path.insert(0, destiny)
            current = prev[destiny]
            
            while current != origin:
                path.insert(0, current)
                current = prev[current]
            
            path.insert(0, current)
        
        return path, distance

    """
        Entrada:
            self: próprio grafo
            origin: origem da busca, é um inteiro como string. Ex: '2', '3'
            destiny: destino da busca, segue o mesmo padrão de origin

        Saída:
            Tupla: (distancia, anteriores)
                distancia: a distância deste menor caminho encontrado
                anteriores: dicionário em que os valores são associados às chaves que eles desembocam
                    Ex: '2': '1' significa que o vértice 2 é acessível através do vértice 1 (1 -> 2)
    """
    def dijkstra(self, origin, destiny = None):
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
        
        if destiny is None: return distances
        return distances[destiny], prev  

fid  = open("input.txt", "r")
g = Graph(fid)
g.graph.edges['2', '3']['distance'] = float('inf')
print(g.graph.edges['2', '3']['distance'])
#print(g.dijkstra('1', '4'))
print(g.getShortestPath('1', '4'))
g.showGraph()