from app import app
from flask import request, render_template


@app.route("/", methods=['GET', 'POST'])
def hello_world():
    streets = [1, 2, 4]
    # [Tempo da viagem, Distância]
    clients = [ [[45.5, 59.3],  [71.5, 89.8],   [122.7, 124.6], [171.5, 189],   [222.7, 224.6]],
                [[76, 32.4],    [89.2, 123],    [94, 213],      [100, 250],     [106, 300]]
              ]
    if request.method == 'GET':
        print('get')
    else:
        if "client-form-submit" in request.form:
            data = from_form_to_client(request)
            print(data)
        elif "car-form-submit" in request.form:
            data = from_form_to_car(request)
            print(data)
        else:
            data = from_form_to_velocity(request)
            print(data)
    
    return render_template('index.html', streets=streets, clients=clients)


def from_form_to_client(request):
    if request.form.get('client-file'):
        client_file = request.form.get('client-file')
        return client_file
    else:
        loc_x = float(request.form.get("input-loc-x"))
        loc_y = float(request.form.get("input-loc-y"))
        dest_x = float(request.form.get("input-dest-x"))
        dest_y = float(request.form.get("input-dest-y"))
        return [loc_x, loc_y, dest_x, dest_y]


def from_form_to_car(request):
    if request.form.get('car-file'):
        car_file = request.form.get('car-file')
        return car_file
    else:
        id_street = int(request.form.get("id_street"))
        return id_street


def from_form_to_velocity(request):
    if request.form.get('velocity-file'):
        velocity_file = request.form.get('velocity-file')
        return velocity_file
    else:
        street = request.form.get('street-id-velocity')
        velocity = float(request.form.get("velocity"))
        return [street, velocity]