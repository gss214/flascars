from operator import imod
from flask import Flask, render_template
from werkzeug.utils import secure_filename
app = Flask(__name__)

from app.controllers import home