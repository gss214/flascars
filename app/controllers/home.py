from app import app
from flask import request, render_template, flash, redirect
from werkzeug.utils import secure_filename
from app.src.graphs.graph import Graph

g = Graph(open('app/files/paa_arquivo.txt'))
g.showGraph()
@app.route("/", methods=['GET', 'POST'])
def hello_world():
    dist = None
    times = None
    value = None
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())
    shortestPaths = None

    if request.method == 'GET':
        print('get')
    else:
        # print(request.form)
        if "client-form-submit" in request.form:
            data = from_form_to_client(request)
            retorno = g.addClient(data['position'], data['destination'])
            g.showGraph()
        elif "car-form-submit" in request.form:
            data = from_form_to_car(request)
            retorno = g.addCar(data['position'], data['edge_id'])
            print(data)
            g.showGraph()
        elif "speed-form-submit" in request.form:
            data = from_form_to_speed(request)
            g.changeSpeed(data['edge_id'], data['speed'])
        elif "id" in request.form:
            value = request.form.get('id')
            shortestPaths = g.clientRoutes(value)
            print('shortest path: ')
            print(shortestPaths)
        elif "idCar" in request.form:
            carValue = int(request.form.get('idCar'))
        elif "select-path" in request.form:
            print(request.form.get('selectedPath'))
            # TODO: Após escolher caminho, o usuário é deletado e o carro é realocado
        elif "view-path" in request.form:
            value = request.form.get('view-path').split('-')[0].strip()
            shortestPaths = g.clientRoutes(value)
            path = request.form.get('view-path').split('-')[1].strip()
            path_array = list(path[1:-1].split(','))
            path_array = [elem.strip()[1:-1] for elem in path_array]
            g.resetColors()
            g.showGraphRoute(path_array, g.clients[value]["approx_position_orig"], g.clients[value]["approx_position_dest"])
            
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())
    if cars != []: 
        carValue = cars[0]
    else:
        carValue = 0
    if not value:
        if clients !=[]:
            value = clients[0]
        else:
            value = 0
    print(clients)
    return render_template('index.html', streets=streets, clients=clients, id_cliente=value, id_car=carValue, cars=cars, dist=dist, time=times, _tuple=[value, carValue], shortestPaths=shortestPaths)


def from_form_to_client(request):
    loc_x = float(request.form.get("input-loc-x"))
    loc_y = float(request.form.get("input-loc-y"))
    dest_x = float(request.form.get("input-dest-x"))
    dest_y = float(request.form.get("input-dest-y"))
    return {"position": (loc_x, loc_y), "destination": (dest_x, dest_y)}

def from_form_to_car(request):
    edge_id = request.form.get("edge-id")
    car_x = request.form.get("car-x")
    car_y = request.form.get("car-y")
    return {"position": (float(car_x), float(car_y)),
            "edge_id": edge_id}


def from_form_to_speed(request):
    street = request.form.get('street-id-speed')
    speed = float(request.form.get("speed"))
    return {"edge_id": street, "speed": speed}


@app.route("/upload_client", methods=['GET','POST'])
def upload_client_list():
    if request.method == 'POST':
        if 'file-client' not in request.files:
            flash('No file part')
            return redirect('/')
        file = request.files['file-client']
        file.readline()
        for line in file:
            values = line.split()
            position = (float(values[1]), float(values[2]))
            destination = (float(values[3]), float(values[4]))
            g.addClient(position, destination)
        g.showGraph()
    return redirect('/')

@app.route("/upload_car", methods=['GET', 'POST'])
def upload_car_list():
    if request.method == 'POST':
        if 'file-car' not in request.files:
            flash('No file part')
            return redirect('/')
        file = request.files['file-car']
        file.readline()
        for line in file:
            values = line.split()
            position = (float(values[1]), float(values[2]))
            edge_id = values[3].decode('utf-8')
            print(values)
            g.addCar(position, edge_id)
        g.showGraph()
    return redirect('/')


@app.route("/upload_speed", methods=['GET', 'POST'])
def upload_speed_list():
    if request.method == 'POST':
        if 'file-speed' not in request.files:
            flash('No file part')
            return redirect('/')
        file = request.files['file-speed']
        file.readline()
        for line in file:
            values = line.split()
            edge_id = values[0].decode('utf-8')
            speed = float(values[1])
            g.changeSpeed(edge_id,speed)
        g.showGraph()
    return redirect('/')

@app.route("/delete/client/<id>", methods=['GET', 'POST'])
def delete_client(id):
    g.removeClient(id)
    g.showGraph()
    return redirect('/')
@app.route("/delete/car/<id>", methods=['GET', 'POST'])
def delete_car(id):
    g.removeCar(id)
    g.showGraph()
    return redirect('/')

@app.route("/selectRoute/<id_client>/<id_car>", methods=['GET', 'POST'])
def select_route(id_client,id_car):
    _ ,rotas = g.getCarRoute(id_client,id_car)
    g.showGraphRoute(rotas,g.graph.nodes[id_car]["orig"], g.clients[id_client]["approx_position_orig"])
    return redirect('/')