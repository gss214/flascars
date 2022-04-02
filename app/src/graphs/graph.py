import networkx as nx
import matplotlib.pyplot as plt
from pyvis import network as net
import time

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
            self.graph.add_node(values[1], x = int(values[2])*50, y = int(values[3])*50)
            self.graph.add_node(values[4], x = int(values[5])*50, y = int(values[6])*50)
            
            # Formata tempo em formato hh:mm
            time_string = time.strftime('%H:%M', time.gmtime((int(values[7])/int(values[8]))*3600))
            edge_title = f"ID = {values[0]}<br>"
            edge_title += f"Speed = {values[8]}km/h<br>"
            edge_title += f"Distance = {values[7]}km<br>"
            edge_title += f"Time = {time_string}"
            
            #str(int(values[7])/int(values[8]))
            self.graph.add_edge(values[1], values[4], distance=values[7], speed=values[8], title=edge_title)

    def showGraph(self):
        graph_plot = net.Network(height='100%', width='100%',notebook=False, directed=True)
        graph_plot.from_nx(self.graph)
        graph_plot.toggle_physics(False)
        graph_plot.toggle_drag_nodes(False)
        graph_plot.show('graph.html')

fid  = open("input.txt", "r")
g = Graph(fid)
g.showGraph()