from app import app
from flask import request, render_template, flash, redirect
from werkzeug.utils import secure_filename
from app.src.graphs.graph import Graph

g = Graph(open('app/files/paa_arquivo.txt'))
@app.route("/", methods=['GET', 'POST'])
def hello_world():
    dist = None
    times = None
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())

    if request.method == 'GET':
        print('get')
    else:
        if "client-form-submit" in request.form:
            data = from_form_to_client(request)
            retorno = g.addClient(data['position'], data['destination'])
        elif "car-form-submit" in request.form:
            data = from_form_to_car(request)
            retorno = g.addCar(data['position'], data['edge_id'])
        elif "id" in request.form:
            value = request.form.get('id')
            dist,times = g.carRoutes(g.clients[value]["approx_node_prev"], g.clients[value]["dist_offset_prev"], g.clients[value]["time_offset_prev"])
        elif "idCar" in request.form:
            carValue = int(request.form.get('idCar'))
        else:
            data = from_form_to_velocity(request)
            retorno = g.changeSpeed(data['edge_id'], data['speed'])
    g.showGraph()
    streets = list(g.edges.keys())
    cars = list(g.cars.keys())
    clients = list(g.clients.keys())
    if cars != []: 
        carValue = cars[0]
    else:
        carValue = 0
    if clients !=[]:
        value = clients[0]
    else :
        value = 0
    return render_template('index.html', streets=streets, clients=clients, id_cliente=value, id_car=carValue, cars=cars, dist=dist, time=times)


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


def from_form_to_velocity(request):
    street = request.form.get('street-id-velocity')
    speed = float(request.form.get("speed"))
    return {"edge_id": street, "speed": speed}


@app.route("/upload_client", methods=['GET', 'POST'])
def upload_client_list():
    if request.method == 'POST':
        if 'file-client' not in request.files:
            flash('No file part')
            return redirect('/')
        file = request.files['file-client']
        file.readline()
        for line in file:
            values = line.split()
            client = {  "position": (float(values[1]), float(values[2])), 
                        "destination": (float(values[3]), float(values[4]))}
            g.addClient(client)
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
            car = {  "position": (float(values[1]), float(values[2])), 
                     "edge_id ": values[3]}
            print(car)
            g.addCar(car)
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
            speed = {"edge_id": values[0], "speed": float(values[1])}
            g.changeSpeed(speed)
    return redirect('/')

@app.route("/delete/client/<id>", methods=['GET', 'POST'])
def delete_client(id):
    g.removeClient(id)
    return redirect('/')
@app.route("/delete/car/<id>", methods=['GET', 'POST'])
def delete_car(id):
    g.removeCar(id)
    return redirect('/')

@app.route("/selectRoute/<id_client>/<id_car>", methods=['GET', 'POST'])
def select_route(id_client,id_car):
    _ ,rotas = g.getCarRoute(id_client,id_car)
    g.showGraphRoute(rotas,g.graph.nodes[id_car]["orig"], g.clients[id_client]["approx_position_dest"], g.clients[id_client]["approx_position_orig"])
    return redirect('/')