from app import app
from flask import request, render_template, flash, redirect
from werkzeug.utils import secure_filename
from app.src.graphs.graph import Graph

g = Graph(open('app/files/paa_arquivo.txt'))
g.showGraph()
global current_path
current_path = None

@app.route("/", methods=['GET', 'POST'])
def hello_world():
    global current_path
    carValue = None
    tempo = None
    distancia = None
    dist = None
    times = None
    value = None
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())
    shortestPaths = None
    path_index = None
    
    if request.method == 'GET':
        print('get')
    else:
        if "client-form-submit" in request.form:
            data = from_form_to_client(request)
            retorno = g.addClient(data['position'], data['destination'])
            g.resetColors()
            g.showGraph()
        elif "car-form-submit" in request.form:
            data = from_form_to_car(request)
            retorno = g.addCar(data['position'], data['edge_id'])
            g.resetColors()
            g.showGraph()
        elif "speed-form-submit" in request.form:
            data = from_form_to_speed(request)
            g.changeSpeed(data['edge_id'], data['speed'])
            g.resetColors()
            g.showGraph()
        elif "id" in request.form:
            value = request.form.get('id')
            distancia, tempo = g.carRoutes(value)
            g.resetColors()
            g.showGraph()
        elif "idCar" in request.form:
            data = request.form.get('idCar').split('-')
            carValue = data[0].strip()
            value = data[1].strip()
            shortestPaths = g.clientRoutes(value)
            car_client_path = g.getCarRoute(value, carValue)
            g.resetColors()
            g.showGraphRoute(car_client_path[1], g.cars[carValue]["position"], g.clients[value]["approx_position_orig"])
        elif "select-path" in request.form:
            # global current_path
            data = request.form.get('select-path').split('-')
            carValue = data[0].strip()
            value = data[1].strip()
            selected_path = int(data[2]) - 1
            g.calcAndShow(carValue, value, current_path[0])
            
            value = None
            g.resetColors()
            g.showGraph()
        elif "view-path" in request.form:
            # global current_path
            carValue = request.form.get('view-path').split('-')[0].strip()
            value = request.form.get('view-path').split('-')[1].strip()
            selected_path = int(request.form.get('view-path').split('-')[2].strip()) - 1
            shortestPaths = g.clientRoutes(value)
            current_path = g.getTotalPath(value, carValue, shortestPaths[selected_path])
            g.resetColors()
            g.showGraphRoute(current_path[1], g.cars[carValue]["position"], g.clients[value]["approx_position_dest"], g.clients[value]["approx_position_orig"])
            path_index = selected_path + 1
            
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())
    if not carValue:
        if cars != []: 
            carValue = cars[0]
        else:
            carValue = 0
    if not value:
        if clients !=[]:
            value = clients[0]
        else:
            value = 0
    metrics = g.getMetrics()
    return render_template('index.html', streets=streets, clients=clients, id_cliente=value, id_car=carValue, cars=cars, 
        dist=dist, time=times, _tuple=[value, carValue], shortestPaths=shortestPaths, path_index=path_index, metrics =metrics, carTime = tempo, carDist = distancia)


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
        g.resetColors()
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
            g.addCar(position, edge_id)
        g.resetColors()
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
        g.resetColors()
        g.showGraph()
    return redirect('/')

@app.route("/delete/client/<id>", methods=['GET', 'POST'])
def delete_client(id):
    g.removeClient(id)
    g.resetColors()
    g.showGraph()
    return redirect('/')
@app.route("/delete/car/<id>", methods=['GET', 'POST'])
def delete_car(id):
    g.removeCar(id)
    g.resetColors()
    g.showGraph()
    return redirect('/')

@app.route("/deletecar/", methods=['GET', 'POST'])
def deletecar():
    id = request.form.get('idCar')
    g.removeCar(id)
    g.resetColors()
    g.showGraph()
    return redirect('/')

@app.route("/selectRoute/<id_client>/<id_car>", methods=['GET', 'POST'])
def select_route(id_client,id_car):
    _ ,rotas = g.getCarRoute(id_client,id_car)
    g.showGraphRoute(rotas,g.graph.nodes[id_car]["orig"], g.clients[id_client]["approx_position_orig"])
    return redirect('/')