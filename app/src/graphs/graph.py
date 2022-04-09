from operator import itemgetter
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
            destination: destino da busca, segue o mesmo padrão de origin

        Saída:
            Tupla: (caminho, distancia)
                caminho: menor caminho encontrado entre a origem e destino
                distancia: a distância deste menor caminho encontrado
    """
    def getShortestPath(self, origin, destination):
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
    def dijkstra(self, origin, destination = None):
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
    def yenkShortestPaths(self, origin, destination, k=5):
        
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
                        weight = g.graph.edges[current_path[j], current_path[j+1]]['time']
                        
                        # caso o peso seja infinito apenas pulamos a iteração
                        if weight == float('inf'):
                            continue
                        
                        # muda temporariamente o peso (tempo de travessia) da aresta para infinito
                        removed_edges.append((current_path[j], current_path[j+1], weight))
                        g.graph.edges[current_path[j], current_path[j+1]]['time'] = float('inf')

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
                    g.graph.edges[edge[0], edge[1]]['time'] = edge[2]
            
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



                    

fid  = open("input3.txt", "r")
g = Graph(fid)
#g.graph.edges['2', '3']['distance'] = float('inf')
#print(g.graph.edges['2', '3']['distance'])
#print(g.dijkstra('1', '4'))
print(g.yenkShortestPaths('1', '13'))
g.showGraph()