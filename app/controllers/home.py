import os
from app import app
from flask import request, render_template, flash, redirect
from werkzeug.utils import secure_filename


@app.route("/", methods=['GET', 'POST'])
def hello_world():
    value = 1
    streets = [1, 2, 4]
    clients = [ [
                {'cost': 10.5, 'distance': 9.2, 'path': ['1', '2', '3', '9', '10', '11', '12', '13']}, 
                {'cost': 2, 'distance': 3.2, 'path': ['1', '2', '4', '13']},
                {'cost': 50, 'distance': 55.2, 'path': ['1', '2', '8', '13']},
                ],[
                {'cost': 58, 'distance': 58.2, 'path': ['1', '2', '3', '9', '10', '11', '12', '13']}, 
                {'cost': 2, 'distance': 5.2, 'path': ['1', '2', '3', '13']},
                {'cost': 59, 'distance': 60.2, 'path': ['1', '2', '3', '13']},
                ],[
                {'cost': 16, 'distance': 8.2, 'path': ['1', '2', '3', '9', '10', '11', '12', '13']}, 
                {'cost': 60, 'distance': 90.2, 'path': ['1', '2', '3', '13']},
                {'cost': 21, 'distance': 20.8, 'path': ['1', '6', '7', '13']}
                ] ]
    if request.method == 'GET':
        print('get')
    else:
        print(request.form)
        if "client-form-submit" in request.form:
            data = from_form_to_client(request)
            print(data)
        elif "car-form-submit" in request.form:
            data = from_form_to_car(request)
            print(data)
        elif "id" in request.form:
            value = int(request.form.get('id'))
        else:
            data = from_form_to_velocity(request)
            print(data)
    
    return render_template('index.html', streets=streets, clients=clients, id_cliente=value)


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
            #TODO addClient(client)
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
            #TODO addCar(car)
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
            #TODO changeSpeed(speed)
    return redirect('/')